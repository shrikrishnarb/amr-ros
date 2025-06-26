FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# 1. Install basic tools & setup locale
RUN apt update && apt install -y \
    locales curl gnupg2 lsb-release wget git sudo software-properties-common \
    && locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 2. Add ROS2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 3. Add Gazebo Fortress repository
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 4. Install ROS2, Gazebo Fortress, and extra useful packages
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-teleop-twist-keyboard \
    ignition-fortress \
    libignition-gazebo6-dev \
    libignition-msgs8-dev \
    libignition-transport11-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    build-essential \
    nano \
    xterm \
    && apt clean

# 5. Source ROS setup in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 6. Working directory
WORKDIR /workspace
