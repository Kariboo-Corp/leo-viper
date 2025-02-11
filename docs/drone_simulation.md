# PX4 Drone Simulation Documentation

This is the documentation for the PX4 Drone simulation. Here, all the links to the proper tutorials are presented with additional specifications when needed.

The goal of this simulation is to have a drone take off vertically, stop at a height of 3 meters, and then move to another point located at the same height, back and forth, for a defined period. This will result in a linear trajectory.

## Tutorials

1. **PX4 and ROS2 Installation**:  
   The tutorial to install and set up PX4 and ROS2:  
   [PX4 & ROS2 Setup](https://docs.px4.io/main/en/ros2/user_guide)

2. **QGroundControl Setup**:  
   The tutorial to install QGroundControl and enable the drone to take off:  
   [QGroundControl Installation](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

3. **Offboard Control Setup**:  
   The tutorial to set up working offboard control:  
   [PX4 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control.html)

Once all the previous requirements are met, here are the steps to have a flying drone in Gazebo:

## Steps to Launch the Simulation

1. **Launch the agent to enable PX4 and ROS2 communication**:
   ```bash
   cd /path/to/micro-ROS-Agent/build
   cd build
   MicroXRCEAgent udp4 -p 8888
2. **Launch QGroundControl in a new terminal:**
    ```
    cd /path/to/QGroundControl.AppImage
    ./QGroundControl.AppImage
    ```
3. **Launch PX4 and the Gazebo simulation in a new terminal:**

    ```
    cd /path/to/PX4-Autopilot
    cd PX4-Autopilot
    make px4_sitl gz_x500
    ``` 
4. **Run the chosen script to control the drone offboard in a new terminal:**
    ```
    cd /path/to/ws_offboard_control/
    cd ~/ws_offboard_control/
    ros2 run px4_ros_com offboard_control.py
    ```