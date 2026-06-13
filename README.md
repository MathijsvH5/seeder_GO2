# Go2 Quadruped Robotics Seeder Project

This repository contains the software stack for an autonomous agricultural seeding system using a Unitree Go2 quadruped. Developed as a Bachelor End Project (BEP), this system bridges high-level ROS 2 autonomy (mapping, Nav2) with low-level C++ locomotion control to deploy legged robots in Relay Cropping (RC) environments.

The architecture utilizes a distributed system: the Go2 handles internal state machines and walking gaits, while broadcasting over a CycloneDDS network to an external computer that processes SLAM, RViz monitoring, and triggers a hardware payload via an ESP32 micro-controller. High-level autonomy and low-level control are not just two options, but two separate things working seamlessly together.

---

## 1. Prerequisites

To run this control station, your machine must meet the following requirements:
* **Operating System:** Ubuntu 22.04 LTS
* **Middleware:** ROS 2 Humble
* **Network:** CycloneDDS (Configured for Wi-Fi discovery bridging with the Go2's internal ROS 2 Foxy system)

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
Install CycloneDDS and the necessary serial libraries for the ESP32 relay:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
pip install pyserial
```

### Network Configuration (Crucial)
Because the Go2 uses an older DDS implementation, you must force your computer to strictly bind to your Wi-Fi interface to prevent dropped packets or `bad_alloc` crashes. 

1. Create a `cyclonedds.xml` file in your home directory:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="[https://cdds.io/config](https://cdds.io/config)" xmlns:xsi="[http://www.w3.org/2001/XMLSchema-instance](http://www.w3.org/2001/XMLSchema-instance)">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>YOUR_WIFI_INTERFACE_NAME</NetworkInterfaceAddress>
        </General>
    </Domain>
</CycloneDDS>
```
2. Add these lines to your `~/.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml
# Uncomment and set this if your Go2 is broadcasting on a specific channel:
# export ROS_DOMAIN_ID=10 
```

---

## 3. Building the Workspaces

This repository builds upon the official Unitree SDK and ROS 2 packages. For detailed installation, dependency management, and core build instructions for the base packages, please refer directly to the official Unitree repositories:

* **Unitree SDK2:** [github.com/unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
* **Unitree ROS2:** [github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

*(Note: Ensure you build any custom ROS 2 packages within your Colcon workspace alongside the official Unitree packages).*

---

## 4. Running the System

Before running any commands, ensure the Go2 is powered on, connected to the same Wi-Fi network as your computer, and the ESP32 seeder mechanism is physically connected.

**1. Source the environment:**
Open a fresh terminal and source both ROS 2 and your local workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/seeder_GO2/unitree_ws/install/setup.bash
```

**2. Launch the High-Level ROS 2 Network (RViz/SLAM):**
```bash
# Example launch command (update with your specific launch file name)
ros2 launch quadruped_ros2_control go2_mapping.launch.py
```

**3. Start the Hardware Relay:**
In a separate terminal, launch the Python script that listens to the ROS network and triggers the ESP32 over USB:
```bash
python3 ~/seeder_GO2/scripts/seed_relay.py
```

**4. Execute the Locomotion Sequence:**
Finally, run the compiled C++ executable that commands the robot to walk to its target, assume the seeding posture, and fire the drop trigger.
```bash
cd ~/seeder_GO2/unitree_sdk_ws/build
./your_sequence_executable_name
```
