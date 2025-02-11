
# 📖 Developer documentation

This is the documentation of the leo-viper repository which is the integration of interbotix xsarm on leo-rover robot using  ROS2.
This is only the half of the project, the other is an other repository which is the ros2 package of the arm itself which follow a given target.

- [📖 Developer documentation](#-developer-documentation)
  - [Project specifications](#project-specifications)
  - [Structure of XSArm package](#structure-of-xsarm-package)
  - [Launch file explanation](#launch-file-explanation)
    - [Duplicated and Adapted Files](#duplicated-and-adapted-files)
    - [Launch Files Created for the Project](#launch-files-created-for-the-project)
    - [Useful parameters](#useful-parameters)
  - [Package Files Detail](#package-files-detail)
  - [Miscs](#miscs)
    - [- Rover moving script :](#--rover-moving-script-)
    - [- Drone target moving :](#--drone-target-moving-)
  - [Useful link](#useful-link)
  - [Future work](#future-work)


## Project specifications

- **Programming language**: Python
- **ROS2 version**: Humble
- **Gazebo version**: 6 Ignition
- **Operating system**: Ubuntu 20.04

> It is recommended to use Docker container to run the project to avoid any dependency issues. But you can also put the ./src folder in your ROS2 workspace and build the package. The Dockerfile is located at the root of this repository.


The core of the project is fusion of two repositories:
- [leo-rover-simulator-ros2](https://github.com/LeoRover/leo_simulator-ros2)
- [xsarm-packages](https://github.com/Interbotix/interbotix_ros_manipulators/tree/humble/interbotix_ros_xsarms)

> Don't forget to switch to **humble** branch when cloning and looking in source code on github because there are major differences between ROS2 versions (even betwen humble and foxy for instance).


The project is a ROS2 package which take the package **interbotix_ros_manipulators** and adapt all its launch files to be compatible with leo-rover simulation. You will can also find **leo_arm_sim.urdf.xacro** which is the fusionned URDF of the arm and the robot.


## Structure of XSArm package

![XSARM package structure (taken from the repository)](./imgs/xsarm_irros_structure.png)


## Launch file explanation

Existing files in the xsarm package were duplicated in the project and adapted accordingly.  

### Duplicated and Adapted Files
- **xsarm_moveit.launch.py**: Launches the MoveIt server for the arm. It also starts the Gazebo simulation. You only need to launch this file.  
- **xsarm_description.launch.py**: Launches the URDF of the arm in RViz for visualization. You don't need to launch it directly.  
- **xsarm_control.launch.py**: Launches the control layer (see diagram), allowing other scripts and nodes to control the arm.  
- **xsarm_ros_control.launch.py**: Launches the ROS control layer, enabling control of the arm via ROS2 messages.  

---

### Launch Files Created for the Project
- **gz_ign.launch.py**: An adaptation of the simulation launcher script for Gazebo Ignition. You don't need to launch it directly, as it is used by other launch files.  
- **xsarm_gz_control.launch.py**: Launches the control layer for the arm in Gazebo, combining **xsarm_control.launch.py** and **gz_ign.launch.py**. You can launch this file along with a control node to operate the arm in Gazebo.  
  - *(Currently a work in progress; further improvements are needed.)*


### Useful parameters

- **robot_model** : The name of the robot model. For the project it would be vx300 but on all tutorial the mobile_px100 is used.
- **hardware_type** : Put **gz_classic** to use gazebo simulator (despite it is ignition).
- **use_sim_time** : Set to true to use simulation time if you are using gazebo.

## Package Files Detail

> For all custom files detailed below, only the *mobile_px100* adaptation has been done for testing. However, the process is the same for other xsarm models.

- **`config/controller`**: Custom controller YAML for the *ign_ros2_control* package. Only *mobile_px100* has been duplicated. You can create your own controller based on the arm you are using (you can find them in the *interbotix_xsarm_sim* package).  
- **`config/srdf/`**: Custom Semantic Robot Description Format for the arm. No major modifications have been made here. 
- **`config/arm_modes.yaml`**: Custom YAML file for the arm modes. Only *mobile_px100* has been duplicated. Nothing changed here, just for testing purposes. 
- **`scripts/`**: Custom scripts for controlling the rover and the drone. See `Misc.` for details.  
- **`urdf/`**: Custom URDF. The base was taken from the Leo Rover xacro, along with a copy of the *mobile_px100* URDF and its control URDF. The control URDF is very important because it details all the joints and the limits of the arm for the *ign_ros2_control* package.
  
## Miscs

### - Rover moving script : 
 You can follow the installation tutorial [here](rover_move.md)
### - Drone target moving : 
You can follow the installation tutorial [here](drone_simulation.md)


## Useful link

[Hub of XSARM ROS2 packages doc](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages.html)
You'll be able to find all parameters that you can use in the launch files and the nodes.

[Gazebo Ros Control Doc](https://control.ros.org/master/doc/gz_ros2_control/doc/index.html) A lot of useful information about how to control a robot in gazebo with ROS2. This is still in work in progress because the xsarms packages are not fully migrated to this packages.

[Understanding Gazebo Architecture](https://articulatedrobotics.xyz/tutorials/ready-for-ros/gazebo/) A good blog page to understand what's going on under the hood of gazebo and ros2 control.

## Future work

- Finish the Gazebo control layer  
- Integrate the second ROS package, which controls the arm to follow a target  
- Finish the second ROS package