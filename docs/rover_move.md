# Leo Rover Simulator - ROS 2 (Humble)

## Description

This project uses the Leo Rover simulation under ROS 2 Humble to control a mobile robot in Gazebo. The robot is simulated to follow programmed commands that allow it to complete a predefined path, such as drawing a square around its starting point.

This project includes:
- A Bash script (`src/scripts/gazebo_rover.sh`) to launch the Gazebo simulation with the rover.
- A Python script (`src/scripts/move_rover.py`) to control the rover's movements.
- A ROS 2-compatible structure for the `leo_simulator-ros2` package.

---

## Prerequisites

Make sure the following are installed:
- **ROS 2 Humble**: https://docs.ros.org/en/humble/index.html
- **Gazebo** (compatible with ROS 2 Humble).
- **Python 3** with the required libraries.
- A properly configured ROS 2 environment.

---

## Installation

### 1. Clone the Leo Rover Repository
Clone the official Leo Rover repository into your ROS 2 workspace:
```bash
cd ~/ros_ws/src
git clone -b humble https://github.com/LeoRover/leo_simulator-ros2.git
```

> If you're using the leo-viper docker environment, the repository is already included in the `/home/ws` directory.

### 2. Compile the Workspace
Compile your ROS 2 workspace:
```bash
cd /home/ws
colcon build
source /home/ws/install/setup.bash
```

### 3. Add Custom Scripts
Add the following files to the root of your ROS 2 workspace (`/home/ws`):
- **`../gazebo_rover.sh`**:
  ```bash
  #!/bin/bash

  # Source ROS 2 workspace
  source ~/ros_ws/install/setup.bash

  # Launch the Leo Rover simulation
  ros2 launch leo_gz_bringup leo_gz.launch.py
  ```
- **`../move_rover.py`**:
  Includes the Python script to move the rover along a predefined path (see below).


> If you're using the leo-viper docker environment, the scripts are already included in the `/home/ws/src/arm_integration/scripts` directory.
### 4. Grant Execution Permissions
Ensure the Bash script is executable:
```bash
chmod +x /home/ws/src/arm_integration/scripts/gazebo_rover.sh
```

---

## Usage

### Step 1: Launch the Simulation
Run the Bash script to start the Gazebo simulation with the rover:
```bash
./gazebo_rover.sh
```

> If you're using the leo-viper docker environment, the script is already included in the `/home/ws/src/arm_integration/scripts` directory.

### Step 2: Control the Rover
Once the Gazebo simulation is running, open another terminal, source ROS 2, and execute the Python script to control the rover:
```bash
source ~/ros_ws/install/setup.bash
python3 ~/ros_ws/move_rover.py
```

The rover should then follow the predefined path.

---

## Rover Operation

The Python script **`move_rover.py`** controls the rover's movements to perform the following actions:
1. **Move forward 1.5 meters.**
2. **Turn 90° (or \( \pi/2 \) radians) to the left.**
3. **Reach a defined perimeter, then trace a square with 3-meter sides.**


## Expected Results

1. The rover moves forward 1.5 meters twice to reach the perimeter.
2. It draws a 3-meter square around its starting point.
3. At the end, the rover comes to a complete stop.

---