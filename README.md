# AMR-ROS: Autonomous Mobile Robot with ROS2, Gazebo, and AI

This project builds a simulated Autonomous Mobile Robot (AMR) in Gazebo using ROS2 Humble, with full SLAM, path planning, and reinforcement learning for AI-based navigation.

---

## Features

- ROS2 Humble based architecture
- Gazebo 3D simulation environment
- SLAM + Path Planning with A*, Dijkstra
- Deep Reinforcement Learning (DQN)
- Object detection using YOLO/OpenCV
- Dockerized for easy setup
- Fully documented

---

## Tech Stack

| Tool/Tech      | Purpose                     |
|----------------|-----------------------------|
| ROS2 Humble    | Robot middleware             |
| Gazebo         | 3D simulation                |
| Python         | Programming language         |
| PyTorch/TensorFlow | AI models (DQN etc.)      |
| Docker         | Containerization             |
| SLAM Toolbox   | Simultaneous localization    |
| RViz2          | Visualization                |

---

## Directory Structure
amr-ros/
├── docker/ # Docker & compose files
├── docs/ # Documentation & architecture
├── media/ # Screenshots, videos
├── src/ # ROS2 workspace

---

## Milestones

- [ ] Setup ROS2 + Gazebo
- [ ] URDF robot model
- [ ] SLAM mapping
- [ ] Navigation with A*
- [ ] RL for path planning
- [ ] Computer vision detection
- [ ] Docker containerization
- [ ] Final video + documentation