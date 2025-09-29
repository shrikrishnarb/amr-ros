FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# 1) Base tools and locale
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release wget git sudo software-properties-common \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 2) Add ROS 2 Humble repository (Ubuntu 22.04 = jammy)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 3) Install ROS 2 Humble (desktop) + Gazebo Classic integration + dev tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    build-essential \
    nano \
    xterm \
    python3-yaml \
    python3-tf-transformations \
    python3-numpy \
    && apt-get clean

# 4) Pip-only python deps (angles)
RUN pip3 install --no-cache-dir angles

# 5) Initialize rosdep (ok to run in image)
RUN rosdep init || true && rosdep update

# 6) Source ROS in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 7) Workspace
WORKDIR /workspace
