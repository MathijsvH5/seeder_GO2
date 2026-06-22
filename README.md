# Go2 Quadruped Robotics Seeder Project

This repository contains the software stack for an autonomous agricultural seeding system using a Unitree Go2 quadruped. Developed as a Bachelor End Project (BEP), this system bridges high-level ROS 2 autonomy (mapping, Nav2) with low-level C++ locomotion control to deploy legged robots in Relay Cropping (RC) environments.

The architecture utilizes a distributed system: the Go2 handles internal state machines and walking gaits, while broadcasting over a CycloneDDS network to an external computer that processes SLAM, RViz monitoring, and triggers a hardware payload via an ESP32 micro-controller. High-level autonomy and low-level control are not just two options, but two separate things working seamlessly together.

---

## SLAM Demonstration



https://github.com/user-attachments/assets/530b2dbc-0e13-4d11-ab9f-96f8a72e8ac6



---

## Sequence Demonstration



https://github.com/user-attachments/assets/9d13696a-381d-4c14-8008-b0a63beb1950

---

## Experiment Trajectory Accuracy


https://github.com/user-attachments/assets/28fbe363-78db-4be1-a4c3-47c8831a7b77


---

## 1. Prerequisites

To run this control station, your machine must meet the following requirements:
* **Operating System:** Ubuntu 22.04 LTS
* **Middleware:** ROS 2 Humble

---

## 2. Installation & Dependencies

### ROS 2 Humble Setup
If you do not have ROS 2 Humble installed, follow the official documentation or run:
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

### Required Dependencies
Install for SLAM:
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```
Install for ESP32 USB connection:
```bash
sudo apt install ros-humble-serial
```

---

## 3. Building the Workspaces

This repository builds upon the official Unitree SDK and ROS 2 packages. For detailed installation, dependency management, and core build instructions for the base packages, please refer directly to the official Unitree repositories:

* **Unitree SDK2:** [github.com/unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
* **Unitree ROS2:** [github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

*(Note: Ensure you build any custom ROS 2 packages within your Colcon workspace alongside the official Unitree packages).*

Build the SDK:
```bash
cd ~/seeder_GO2/unitree_sdk_ws/build
make
```

Build ROS 2 packages:
```bash
cd ~/seeder_GO2/unitree_ws
colcon build
source install/setup.bash
```

---

## 4. Running the System

The system can be run in two different modes depending on your testing requirements. Before running either mode, ensure the Go2 is powered on.

### Mode A: Standalone Planting Sequence (Wireless / SSH)
If you only want to test the locomotion and planting sequence without SLAM mapping or the physical ESP32 trigger, you can run this entirely wirelessly directly on the robot's onboard computer.

**1. Connect to the Robot:**
Open a terminal and establish a wireless SSH connection (code: 123):
```bash
ssh unitree@192.168.6.111
```

**2. Execute the Sequence:**
Navigate to the SDK build folder and run the `fast_planting` executable, binding it to the robot's internal `eth0` interface:
```bash
cd ~/seeder_GO2/unitree_sdk_ws/build
./fast_planting_no_esp eth0
```

---

### Mode B: Planting with ESP32 Integration (Cabled Connection Required)
To run the full autonomous stack—including ESP32 seed deployment, your computer must be physically tethered to the Go2 via Ethernet, and the ESP32 must be connected via Bluetooth.

**1. Source the environment (On your tethered laptop):**
Open a fresh terminal on your laptop and source both ROS 2 and your local workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/seeder_GO2/unitree_ws/install/setup.bash
```

**2. Start the Hardware Relay:**
In a separate terminal, launch the script that connects the esp to the laptop via bluetooth:
```bash
./connect_dog.sh
```

**3. Execute the Full Locomotion Sequence:**
Finally, run the compiled C++ executable on your laptop to start the integrated sequence:
```bash
cd ~/seeder_GO2/unitree_sdk_ws/build
./fast_planting eno1
```

### Mode C: SLAM (Cabled Connection Required)

**1. Source the environment (On your tethered laptop):**
Open a fresh terminal on your laptop and source both ROS 2 and your local workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/seeder_GO2/unitree_ws/install/setup.bash
```

**2. Launch the SLAM Network:**
Launch the SLAM and localization nodes to begin mapping:
```bash
ros2 launch unitree_ros2_example demo_localization_launch.py
```
**3. Launch RVIZ:**
```bash
ros2 run rviz2 rviz2
```

