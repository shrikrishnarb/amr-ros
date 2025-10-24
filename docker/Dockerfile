# docker/Dockerfile
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# Base tools + ROS dev tools + GUI bits
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential cmake git wget curl \
    python3-pip python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-dev-tools \
    ros-humble-rviz2 ros-humble-xacro ros-humble-gazebo-ros-pkgs \
    gazebo \
    libgl1-mesa-glx libx11-6 libxext6 \
    # Explicit runtime deps to avoid surprises
    ros-humble-angles \
    ros-humble-tf-transformations \
    python3-numpy python3-yaml \
 && rm -rf /var/lib/apt/lists/*

# ---- rosdep (install deps declared in package.xml) ----
# Copy only manifests to leverage Docker layer caching
WORKDIR /tmp/manifests
COPY colcon_ws/src/amr_description/package.xml /tmp/manifests/amr_description/package.xml
# Initialize & run rosdep (handle images where init isn't pre-run)
RUN apt-get update && apt-get install -y ros-humble-nav2-simple-commander || true
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
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]