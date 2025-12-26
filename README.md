# Human Detection using YOLOv8 with PX4 and Gazebo

This project demonstrates **human detection in a PX4–Gazebo simulation environment** using **YOLOv8**.
A simulated multirotor equipped with a camera flies in a **forest world**, streams images through a **Gazebo–ROS2 bridge**, and performs real-time human detection using a YOLOv8 ROS2 node.

---

## Overview

**Pipeline:**

Gazebo (Forest World)  
→ PX4 SITL Multirotor  
→ Camera Sensor  
→ Gazebo–ROS2 Image Bridge  
→ YOLOv8 ROS2 Node  
→ Human Detection Output  

This setup is useful for:
- Perception research in UAVs  
- Vision-based autonomy testing  
- PX4 + ROS2 + Gazebo integration  

---

## Software Stack

- **PX4 Autopilot (SITL)**
- **Gazebo (gz-sim)**
- **ROS 2**
- **YOLOv8 (Ultralytics)**
- **Python 3.10**

---

## Repository Contents

```text
├── custom_trajectory.py      # Example trajectory / control script
├── commands.txt              # Reference commands (this README is derived from it)
├── README.md


## Running the Simulation

This section describes how to run the PX4–Gazebo simulation and perform
real-time human detection using YOLOv8.

---

### 1️⃣ (Optional) Run custom trajectory script

```bash
python3.10 custom_trajectory.py

2️⃣ Spawn the forest environment

Set the Gazebo model path:

export GZ_SIM_RESOURCE_PATH=$HOME/Documents/ARMY_PROJECT/PX4-Autopilot/Tools/simulation/gz/models

Launch the forest world:

gz sim ~/Documents/ARMY_PROJECT/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf

3️⃣ Spawn the PX4 drone (SITL)

make px4_sitl gz_x500_<variant>

Example (depth camera enabled):

make px4_sitl gz_x500_depth

4️⃣ Run PX4 + Gazebo together (recommended)

make px4_sitl gz_x500_depth PX4_GZ_WORLD=forest

5️⃣ Inspect Gazebo topics (debugging)

gz topic -l

Camera to ROS 2 Bridge
6️⃣ Bridge Gazebo camera image to ROS 2

ros2 run ros_gz_image image_bridge \
/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/image \
/camera/image_raw

7️⃣ Verify camera feed

List ROS 2 topics:

ros2 topic list

Check image rate:

ros2 topic hz /camera/image_raw

Visualize the camera stream:

ros2 run rqt_image_view rqt_image_view

YOLOv8 Setup and Execution
8️⃣ Build YOLOv8 ROS 2 package

cd ~/ros2_ws
colcon build --symlink-install --packages-select yolov8_ros
source install/setup.bash

9️⃣ Run YOLOv8 detection node

ros2 run yolov8_ros yolov8_node

The node subscribes to /camera/image_raw and performs real-time
human detection in the Gazebo simulation.

