# AMR-ROS: Multi-AGV Simulation with ROS 2, Gazebo, and Fleet Management

This project provides a **complete simulation framework** for **Autonomous Mobile Robots (AMRs)** using **ROS 2 Humble** and **Gazebo Classic**, featuring **multi-AGV orchestration**, **fleet management**, **battery monitoring**, and **dynamic task allocation**.

---

## Features

- **Multi-AGV Simulation** in Gazebo with ROS 2 namespaces
- **Fleet Manager** for:
  - Dynamic **task allocation** (pickup → drop-off)
  - **Battery monitoring** and **automatic charging**
- **Waypoint Navigation** using ground-truth odometry
- **Obstacle detection & smooth stop** using LiDAR
- **Charging station docking monitor**
- **Configurable launch system** for spawning multiple AGVs and loading tasks from YAML
- **Dockerized environment** for reproducibility and easy setup

---

## Tech Stack

| Tool/Tech         | Purpose                                  |
|-------------------|------------------------------------------|
| **ROS 2 Humble**  | Robot middleware                        |
| **Gazebo Classic**| 3D simulation environment               |
| **Python**        | ROS 2 nodes (fleet manager, controllers)|
| **colcon**        | ROS 2 build system                      |
| **Docker**        | Containerization for portability        |
| **RViz2**         | Visualization                           |

---

## Directory Structure

amr-ros/   
├── docker/ # Dockerfile and scripts   
├── colcon_ws/ # ROS 2 workspace   
│ ├── src/amr_description/ # Nodes, launch files, configs   
│ └── ...   
│── docs/   
│── media/ #sereenshots, images, etc.   
└── README.md  

---

## Core Components
- **ground_truth_waypoint_follower.py**  
  Executes dynamic goals (`/<ns>/goal_pose`) → publishes `cmd_vel` with obstacle-aware smooth stopping.
- **obstacle_detection_node.py**  
  Monitors LiDAR scan → publishes `/<ns>/obstacle_detected`.
- **fleet_manager.py**  
  Assigns tasks from `tasks.yaml`, monitors battery, sends goals to AGVs.
- **charger_dock_monitor.py**  
  Detects proximity to charging stations → publishes `/<ns>/on_charger`.

---

## Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/shrikrishnarb/amr-ros.git
cd amr-ros
```

### 2. Build Docker Image
From the docker/ directory:
```bash
cd docker
docker build -t amr-ros-dev .
```

### 3. Run the Container
Enable X11 for GUI (Gazebo, RViz):
```bash
xhost +local:root
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/../:/workspace \
    --name amr-dev \
    amr-ros-dev
```

### 4. Inside the Container
```bash
# Source ROS
source /opt/ros/humble/setup.bash

# Build the workspace
cd /workspace/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the Simulation
