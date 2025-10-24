# docker/Dockerfile
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
# Locale (optional), basic tools, ROS dev tools, Gazebo Classic, RViz, xacro
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential cmake git wget curl \
    python3-pip python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-dev-tools \
    ros-humble-rviz2 ros-humble-xacro ros-humble-gazebo-ros-pkgs \
    gazebo \
    libgl1-mesa-glx libx11-6 libxext6 \
 && rm -rf /var/lib/apt/lists/*

# Optional Python libs commonly used by AMR projects (adjust as needed)
RUN pip3 install --no-cache-dir numpy pyyaml

# Create a non-root user so mounted files aren’t owned by root on the host
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

# Workspace location expected by the README steps
WORKDIR /workspace/colcon_ws
RUN mkdir -p src

# Lightweight entrypoint: source ROS + workspace if present; set X11 var
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]