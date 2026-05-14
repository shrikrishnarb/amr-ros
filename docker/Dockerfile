# docker/Dockerfile
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3
ENV DISPLAY=:0

# Base tools + ROS dev tools + GUI bits
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential cmake git wget curl xterm \
    python3-pip python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-dev-tools \
    ros-humble-rviz2 ros-humble-xacro ros-humble-gazebo-ros-pkgs \
    gazebo \
    libgl1-mesa-glx libx11-6 libxext6 \
    # Motion / transform deps
    ros-humble-angles \
    ros-humble-tf-transformations \
    ros-humble-tf2-ros \
    python3-numpy python3-yaml \
    # Navigation stack
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-nav2-map-server \
    # SLAM
    ros-humble-slam-toolbox \
    # Teleop
    ros-humble-teleop-twist-keyboard \
    # Robot state / joint state publishers
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
 && rm -rf /var/lib/apt/lists/*

# ---- Fix transforms3d NumPy incompatibility (np.float removed in NumPy 1.24+) ----
RUN python3 -c "
import pathlib
path = pathlib.Path('/usr/lib/python3/dist-packages/transforms3d/quaternions.py')
if path.exists():
    content = path.read_text()
    content = content.replace(
        '_MAX_FLOAT = np.maximum_sctype(np.float)',
        '_MAX_FLOAT = np.finfo(np.float64).max'
    )
    path.write_text(content)
    print('transforms3d patch applied')
else:
    print('transforms3d not found at expected path, skipping patch')
"

# ---- Install ultralytics and pin NumPy compatible with both ultralytics and transforms3d ----
# NumPy 1.24+ removes np.float which breaks transforms3d; pin below 1.24 as primary fix,
# the file patch above is a safety net.
RUN pip3 install --no-cache-dir \
    "numpy>=1.21,<1.24" \
    ultralytics \
    && python3 -c "import ultralytics; print('ultralytics OK')" \
    && python3 -c "from tf_transformations import euler_from_quaternion; print('tf_transformations OK')"

# ---- rosdep (install deps declared in package.xml) ----
# Copy only manifests to leverage Docker layer caching
WORKDIR /tmp/manifests
COPY colcon_ws/src/amr_description/package.xml /tmp/manifests/amr_description/package.xml
# Initialize & run rosdep (handle images where init isn't pre-run)
RUN rosdep update || (rosdep init && rosdep update) && \
    rosdep install --from-paths /tmp/manifests --ignore-src -r -y

# ---- Create non-root user (unchanged) ----
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

# Workspace (mounted later)
WORKDIR /workspace/colcon_ws
RUN mkdir -p src

# Entry point
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# ---- Verify all critical dependencies installed correctly ----
RUN python3 -c "
import numpy; print(f'NumPy {numpy.__version__} OK')
import ultralytics; print('ultralytics OK')
from tf_transformations import euler_from_quaternion; print('tf_transformations OK')
import transforms3d; print('transforms3d OK')
"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]