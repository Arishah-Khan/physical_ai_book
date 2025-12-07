---
sidebar_position: 0
title: "Prerequisites & Setup Guide"
description: "Complete setup guide for the humanoid robotics development environment"
---

# Prerequisites & Setup Guide

## System Requirements

### Hardware Requirements
- **Processor**: Multi-core processor (Intel i5/i7 or equivalent AMD)
- **Memory**: 8GB+ RAM (16GB+ recommended for simulation)
- **Storage**: 20GB+ free disk space
- **Graphics**: GPU with OpenGL 3.3+ support (for visualization)
- **Network**: Internet connection for package installation

### Operating System
- **Ubuntu 22.04 LTS** (recommended for ROS 2 Humble Hawksbill)
- **Alternative**: Docker container with Ubuntu 22.04

## Software Installation

### ROS 2 Humble Hawksbill
ROS 2 Humble Hawksbill is the LTS version recommended for humanoid robotics.

```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base

# Install colcon build tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Python Dependencies
```bash
# Install Python 3.10+ and pip
sudo apt update
sudo apt install python3-dev python3-pip

# Install essential Python packages
pip3 install -U colcon-common-extensions
pip3 install -U rosdep rosinstall_generator vcstool
pip3 install -U numpy scipy matplotlib
pip3 install -U opencv-python cv-bridge
pip3 install -U transforms3d
```

### Development Tools
```bash
# Install Git
sudo apt install git

# Install essential build tools
sudo apt install build-essential cmake pkg-config

# Install visualization tools
sudo apt install python3-ros-visualisation
sudo apt install ros-humble-rviz2
```

## Development Environment Setup

### 1. Create ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the empty workspace to generate setup files
colcon build
```

### 2. Set Environment Variables
Add these lines to your `~/.bashrc` file:
```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Set ROS Domain ID (optional, for network isolation)
export ROS_DOMAIN_ID=42
```

### 3. Verify Installation
```bash
# Check ROS 2 installation
ros2 --version

# Test ROS 2 with a simple example
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
# In another terminal: ros2 run demo_nodes_cpp listener
```

## Specialized Tools for Humanoid Robotics

### 1. Gazebo Simulation
```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt install ros-humble-gazebo-*
sudo apt install ignition-garden
```

### 2. MoveIt! for Motion Planning
```bash
# Install MoveIt! for humanoid manipulation
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-visual-tools
```

### 3. Navigation Stack
```bash
# Install navigation stack for humanoid navigation
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### 4. Perception Packages
```bash
# Install perception packages
sudo apt install ros-humble-vision-opencv
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-compressed-image-transport
```

## Docker Setup (Alternative)

If you prefer using Docker for isolation:

```bash
# Install Docker
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER

# Pull ROS 2 Humble Docker image
docker pull osrf/ros:humble-desktop-full

# Run container with GUI support
xhost +
docker run -it \
  --name ros2_humble \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  osrf/ros:humble-desktop-full
```

## Verification Steps

### 1. Check ROS 2 Installation
```bash
# Verify ROS 2 installation
ros2 topic list
ros2 service list
ros2 action list
```

### 2. Test Basic Functionality
```bash
# Terminal 1: Start a talker
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Start a listener
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

### 3. Test rclpy
```python
# Create a test Python script
cat > test_rclpy.py << EOF
import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello from rclpy!')

def main():
    rclpy.init()
    node = TestNode()
    node.get_logger().info('Node created successfully!')
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Run the test
python3 test_rclpy.py
```

## Troubleshooting Common Issues

### 1. Permission Errors
```bash
# If you get permission errors with Docker
sudo usermod -aG docker $USER
# Log out and log back in
```

### 2. Package Installation Issues
```bash
# Update package lists
sudo apt update

# Fix broken dependencies
sudo apt --fix-broken install

# Clean package cache
sudo apt autoremove && sudo apt autoclean
```

### 3. ROS Environment Issues
```bash
# Always source the ROS environment
source /opt/ros/humble/setup.bash

# Or add to your .bashrc file permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Optional: IDE Setup

### VS Code with ROS Extension
```bash
# Install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# Install ROS extension
code --install-extension ms-iot.vscode-ros
```

## Next Steps

Once you have completed the setup:

1. [Continue to Module 1: The Robotic Nervous System (ROS 2)](./module1/chapter01-introduction-to-ros2.md)
2. Review the [Learning Roadmap](./roadmap.md) to understand the suggested learning path
3. Join the community for support and discussions

## Support

- Check the ROS 2 documentation: [docs.ros.org](https://docs.ros.org/)
- Visit the ROS answers: [answers.ros.org](https://answers.ros.org/)
- Join the ROS discourse: [discourse.ros.org](https://discourse.ros.org/)