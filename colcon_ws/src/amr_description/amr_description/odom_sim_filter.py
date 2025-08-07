#!/usr/bin/env python3

from copy import deepcopy
from math import cos, sin

import numpy as np
import rclpy
import rclpy.logging
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_inverse, quaternion_matrix, quaternion_multiply


def as_np(msg: Point | Quaternion) -> np.ndarray:
  if isinstance(msg, Point):
    return np.array([msg.x, msg.y, msg.z])
  if isinstance(msg, Quaternion):
    return np.array([msg.x, msg.y, msg.z, msg.w])
  raise TypeError(f"Type {type(msg)} is not supported.")


def quaternion_sub(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
  """Returns `lhs` - `rhs` quaternion calculation result."""
  return np.array(quaternion_multiply(lhs, quaternion_inverse(rhs)))


class OdomSimFilter_2(Node):
  """Overwrite odometry by the calculated twist and pose relative to virtual origin."""

  def __init__(self):
    """Init."""
    super().__init__("odom_sim_filter")
    # Declare the robot_namespace parameter
    self.declare_parameter('robot_namespace', 'default_namespace')
    # Get the robot_namespace parameter value
    self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

    # Log the namespace to check the value
    self.get_logger().info(f"Robot namespace is: {self.robot_namespace}")

    # Instance attribute
    self.latest_timestamp: float = -1.0
    self.latest_position: np.ndarray = np.array([0.0, 0.0, 0.0])
    self.latest_orientation: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0])
    self.origin_position: np.ndarray = np.array([0.0, 0.0, 0.0])
    self.origin_orientation: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0])
    self.origin_rotation_matrix: np.ndarray = np.eye(4, 4)

    # Parameter
    self.odom_frame = self.declare_parameter("odom_frame", f"{self.robot_namespace}/odom").value
    self.base_frame = self.declare_parameter("base_frame", f"{self.robot_namespace}/base_footprint").value
    self.odom_jump_threshold = self.declare_parameter("odom_jump_threshold", 1.0).value

    # Publisher
    self.odom_pub = self.create_publisher(Odometry, f"/{self.robot_namespace}/diff_drive_controller/odom", 10)
    self.tf_broadcaster = TransformBroadcaster(self)

    # Subscriber
    self.odom_sub = self.create_subscription(Odometry, f"/{self.robot_namespace}/odom", self.odom_cb, 10)

    # Service
    self.reset_odometry_sub = self.create_service(
      Empty, f"/{self.robot_namespace}/diff_drive_controller/reset_odometry", self.reset_odometry_cb
    )

  def odom_cb(self, msg: Odometry):
    """Callback for /{self.robot_namespace}/diff_drive_controller/odom."""
    if self.latest_timestamp < 0.0:
      # This is the first message. Start calculation from next message.
      self.latest_timestamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
      self.latest_position = as_np(msg.pose.pose.position)
      self.latest_orientation = as_np(msg.pose.pose.orientation)
      return

    current_timestamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
    current_position = as_np(msg.pose.pose.position)
    current_orientation = as_np(msg.pose.pose.orientation)
    dt = current_timestamp - self.latest_timestamp

    # Do not calculate twist when time jump detected.
    if dt > 0 and dt < self.odom_jump_threshold:
      position_on_origin = self.origin_rotation_matrix @ np.hstack([current_position - self.origin_position, 1])
      quaternion_on_origin = quaternion_sub(current_orientation, self.origin_orientation)
      calculated_twist = self.twist2d_from_pose_diff(
        current_position, current_orientation, self.latest_position, self.latest_orientation, dt
      )

      # Overwrite odometry
      filtered_msg = deepcopy(msg)
      filtered_msg.header.frame_id = self.odom_frame
      filtered_msg.pose.pose.position.x = position_on_origin[0]
      filtered_msg.pose.pose.position.y = position_on_origin[1]
      filtered_msg.pose.pose.position.z = position_on_origin[2]
      filtered_msg.pose.pose.orientation.x = quaternion_on_origin[0]
      filtered_msg.pose.pose.orientation.y = quaternion_on_origin[1]
      filtered_msg.pose.pose.orientation.z = quaternion_on_origin[2]
      filtered_msg.pose.pose.orientation.w = quaternion_on_origin[3]
      filtered_msg.twist.twist = calculated_twist
      self.odom_pub.publish(filtered_msg)

      # Build transform
      odom_to_base = TransformStamped()
      odom_to_base.header.frame_id = self.odom_frame
      odom_to_base.header.stamp = filtered_msg.header.stamp
      odom_to_base.child_frame_id = self.base_frame
      odom_to_base.transform.translation.x = filtered_msg.pose.pose.position.x
      odom_to_base.transform.translation.y = filtered_msg.pose.pose.position.y
      odom_to_base.transform.translation.z = filtered_msg.pose.pose.position.z
      odom_to_base.transform.rotation = filtered_msg.pose.pose.orientation
      self.tf_broadcaster.sendTransform(odom_to_base)

    self.latest_timestamp = current_timestamp
    self.latest_position = current_position
    self.latest_orientation = current_orientation

  def reset_odometry_cb(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
    """Callback for /{self.robot_namespace}/diff_drive_controller/reset_odometry."""
    if self.latest_timestamp >= 0:
      self.origin_position = self.latest_position
      self.origin_orientation = self.latest_orientation
      self.origin_rotation_matrix = quaternion_matrix(quaternion_inverse(self.origin_orientation))
    return res

  def twist2d_from_pose_diff(
    self,
    current_position: np.ndarray,
    current_orientation: np.ndarray,
    latest_position: np.ndarray,
    latest_orientation: np.ndarray,
    dt: float,
  ) -> Twist:
    """Calculate 2D Twist from pose difference."""
    twist = Twist()
    pos_diff = current_position - latest_position
    yaw = euler_from_quaternion(current_orientation)[2]
    # Calculate cosine similarity between AGV orientation and actual travel.
    # It should be 1 (moving forward) or -1 (moving backward)
    lin_x_sign = 1 if pos_diff[0] * cos(yaw) + pos_diff[1] * sin(yaw) > 0 else -1
    twist.linear.x = lin_x_sign * np.linalg.norm(pos_diff) / dt
    quat_diff = quaternion_sub(current_orientation, latest_orientation)
    twist.angular.z = euler_from_quaternion(quat_diff)[2] / dt
    return twist


def main(args=None):
  rclpy.init(args=args)
  node = OdomSimFilter_2()
  rclpy.spin(node)


if __name__ == "__main__":
  main()
