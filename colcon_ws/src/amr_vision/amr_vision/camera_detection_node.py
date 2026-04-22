#!/usr/bin/env python3
"""
camera_detection_node.py

YOLOv8-based camera perception node for the amr_vision fleet.

Subscribes to /{ns}/camera/image_raw and publishes:
  - /{ns}/camera/detections        (std_msgs/String, JSON per frame)
  - /{ns}/camera/obstacle_semantic (std_msgs/String, one of:
                                    "clear" | "person" | "pallet" | "unknown_obstacle")

Inference runs on a background thread at up to 5 Hz to avoid blocking
the ROS2 spin loop.
"""

import json
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore[assignment,misc]

# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

# Maps COCO class IDs to semantic labels used by the fleet manager.
# "suitcase" (class 28) proxies for "pallet" in simulation — no pallet
# class exists in COCO, but suitcases share a similar rectangular footprint.
YOLO_CLASS_MAP: dict[int, str] = {
    0: "person",
    28: "pallet",
}

# Classes that count as obstacles for the obstacle_semantic topic.
OBSTACLE_CLASSES: set[str] = {"person", "pallet"}

# Semantic priority order (highest index wins when multiple classes detected).
_SEMANTIC_PRIORITY: list[str] = ["clear", "unknown_obstacle", "pallet", "person"]


def _highest_priority_semantic(class_names: list[str]) -> str:
    """Return the highest-priority semantic label from a list of detected class names."""
    best = "clear"
    for name in class_names:
        if name in OBSTACLE_CLASSES:
            candidate = name
        else:
            candidate = "unknown_obstacle"
        if _SEMANTIC_PRIORITY.index(candidate) > _SEMANTIC_PRIORITY.index(best):
            best = candidate
    return best


class CameraDetectionNode(Node):
    """Runs YOLOv8 inference on camera frames and publishes semantic detections."""

    def __init__(self) -> None:
        super().__init__("camera_detection_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("namespace", "agv1")

        model_path: str = str(self.get_parameter("model_path").value)
        self._conf_threshold: float = float(self.get_parameter("conf_threshold").value)
        self._ns: str = str(self.get_parameter("namespace").value)

        # ------------------------------------------------------------------
        # YOLO model
        # ------------------------------------------------------------------
        self._model: Optional[object] = None
        if YOLO is None:
            self.get_logger().error(
                "ultralytics package not installed — camera_detection_node will not run inference."
            )
        else:
            try:
                self._model = YOLO(model_path)
                self.get_logger().info(
                    f"[{self._ns}] YOLO model loaded: {model_path}"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"[{self._ns}] Failed to load YOLO model '{model_path}': {exc}"
                )
                self._model = None

        # ------------------------------------------------------------------
        # CV bridge and threading state
        # ------------------------------------------------------------------
        self._bridge: CvBridge = CvBridge()
        self._inference_lock: threading.Lock = threading.Lock()

        # Rate limiting — process at most 5 frames per second
        self._last_inference_time: float = 0.0
        self._min_interval: float = 0.2  # seconds (1 / 5 Hz)

        # Track last published semantic to suppress redundant INFO logs
        self._last_semantic: str = "clear"

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._detections_pub = self.create_publisher(
            String, f"/{self._ns}/camera/detections", 10
        )
        self._semantic_pub = self.create_publisher(
            String, f"/{self._ns}/camera/obstacle_semantic", 10
        )

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        self.create_subscription(
            Image,
            f"/{self._ns}/camera/image_raw",
            self._image_callback,
            10,
        )

        self.get_logger().info(
            f"CameraDetectionNode ready | ns={self._ns} | model={model_path}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        """Receive a camera frame; rate-limit and dispatch inference to a thread."""
        now = time.time()
        if now - self._last_inference_time < self._min_interval:
            return
        if self._model is None:
            return

        threading.Thread(
            target=self._run_inference,
            args=(msg,),
            daemon=True,
        ).start()
        self._last_inference_time = now

    # ------------------------------------------------------------------
    # Inference (runs on background thread)
    # ------------------------------------------------------------------

    def _run_inference(self, msg: Image) -> None:
        """Convert image, run YOLO, publish detections and semantic label."""
        # Non-blocking acquire — if another inference is already running, skip frame.
        acquired = self._inference_lock.acquire(blocking=False)
        if not acquired:
            return

        try:
            # Convert ROS Image to BGR numpy array
            cv_image: np.ndarray = self._bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run YOLO inference
            start: float = time.perf_counter()
            results = self._model(  # type: ignore[misc]
                cv_image,
                conf=self._conf_threshold,
                verbose=False,
            )
            inference_ms: float = (time.perf_counter() - start) * 1000.0

            # ------------------------------------------------------------------
            # Parse detections
            # ------------------------------------------------------------------
            detections: list[dict] = []
            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                for i in range(len(boxes)):
                    cls_id = int(boxes.cls[i].item())
                    if cls_id not in YOLO_CLASS_MAP:
                        continue
                    conf = float(boxes.conf[i].item())
                    xyxy = boxes.xyxy[i].tolist()
                    bbox = [int(v) for v in xyxy]
                    detections.append({
                        "class_name": YOLO_CLASS_MAP[cls_id],
                        "confidence": round(conf, 2),
                        "bbox": bbox,
                    })

            # Log detected classes at DEBUG level (avoids per-frame INFO spam)
            if detections:
                detected_names = [d["class_name"] for d in detections]
                self.get_logger().debug(
                    f"[{self._ns}] Detected: {detected_names} "
                    f"({inference_ms:.1f} ms)"
                )

            # ------------------------------------------------------------------
            # Build and publish detections JSON
            # ------------------------------------------------------------------
            payload: dict = {
                "timestamp": time.time(),
                "detections": detections,
                "inference_ms": round(inference_ms, 1),
                "frame_id": f"{self._ns}/camera_link",
            }
            det_msg = String()
            det_msg.data = json.dumps(payload)
            self._detections_pub.publish(det_msg)

            # ------------------------------------------------------------------
            # Determine and publish semantic label
            # ------------------------------------------------------------------
            detected_class_names = [d["class_name"] for d in detections]
            semantic: str = _highest_priority_semantic(detected_class_names)

            if semantic != self._last_semantic:
                self.get_logger().info(
                    f"[{self._ns}] Semantic change: {self._last_semantic} → {semantic}"
                )
                self._last_semantic = semantic

            sem_msg = String()
            sem_msg.data = semantic
            self._semantic_pub.publish(sem_msg)

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"[{self._ns}] Inference error: {exc}"
            )
        finally:
            self._inference_lock.release()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CameraDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
