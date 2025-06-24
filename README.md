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

---

## Installation Procedure

### Installing WSL (for Windows)
1. Open a command prompt in **Administrator mode** and run the following command:
    ```bash
    wsl --install
    ```

2. Open Ubuntu from the Start menu and set your username and password in Ubuntu.  
   You need to set it here before opening it from Windows Terminal.

#### Starting Ubuntu (WSL)
1. Start Windows Terminal.
2. Select the **Ubuntu** tab from the `+` button to the right of the `↓` symbol.

### Install Docker on Ubuntu
Install Docker Engine on Ubuntu by following the official Docker installation steps. Here are the commands:

1. Update package lists and install prerequisites:
    ```bash
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    ```

2. Install the Docker GPG key:
    ```bash
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    ```

3. Add Docker's official repository:
    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    ```

4. Install Docker packages:
    ```bash
    sudo apt-get update
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

5. Verify Docker installation by running:
    ```bash
    sudo docker run hello-world
    Hello from Docker!
    This message shows that your installation appears to be working correctly.

    To generate this message, Docker took the following steps:
    1. The Docker client contacted the Docker daemon.
    2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
       (amd64)
    3. The Docker daemon created a new container from that image which runs the
       executable that produces the output you are currently reading.
    4. The Docker daemon streamed that output to the Docker client, which sent it
       to your terminal.

    ```

### Advanced Preparation

#### Add the Development Account to the Docker Group
You need to be able to run Docker commands without using `sudo`. To check whether you can run commands without `sudo`, run:

```bash
# Example output when sudo is required for execution
docker ps
permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Get "http://%2Fvar%2Frun%2Fdocker.sock/v1.46/containers/json": dial unix /var/run/docker.sock: connect: permission denied
```
```bash
# Example output when sudo is not required for execution
docker ps
CONTAINER ID   IMAGE     COMMAND   CREATED   STATUS    PORTS     NAMES
```

You can run sudo without adding the account to the docker group docker. To add the account to the docker group, run the following command.
```bash
sudo gpasswd -a $USER docker
```

Please log out and log back in to reflect the group addition. Try running it again and 
make sure it can be run without logging in .docker ps sudo docker

### Clone the repository from GitHub
```bash
git clone https://github.com/shrikrishnarb/amr-ros.git
```

### Build and Run the Docker Container

From the docker/ directory, build your Docker image:

```bash
cd amr-ros/docker
docker build -t amr-ros-dev .
```

Run the container interactively:

```bash
docker run -it --rm --name amr-dev \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/../:/workspace \
    amr-ros-dev
```

Verify Installation
Inside the container:
```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
ign gazebo --version
```

You should see:

humble  
Gazebo Sim, version 6.17.0  
Copyright (C) 2018 Open Source Robotics Foundation.  
Released under the Apache 2.0 License.  
