This is the documentation for the PX4 Drone simulation.

Here are presented all the links to the proper tutorials with some added specifications when needed.

The goal of this simulation is to have a drone take off vertically, stop at a height of 3 meters and then go from there to another point located at the same height back and forth for a defined period, resulting in a linear trajectory.

The tutorial to install and set-up PX4 and ROS2: https://docs.px4.io/main/en/ros2/user_guide

Then, the tutorial to install QGroundControl and have the drone be able to take off: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

From that point, we should have a functionning Gazebo simulation with the drone in it. Some simple commands such as "take off" or "land" using the commander should be executed.

At last, the tutorial to have a working offboard control: https://docs.px4.io/main/en/ros2/offboard_control.html

Once all the previous requirements are met, here are all the steps to have a flying drone in Gazebo:

-Open a terminal and launch the agent to have PX4 and ROS2 communicate:

    cd Micro-XRCE-DDS-Agent
    cd build
    MicroXRCEAgent udp4 -p 8888

-(new terminal) launch QGC:

    cd
    ./QGroundControl.AppImage

-(new terminal) launch PX4 and the Gazebo simulation:

    cd
    cd PX4-Autopilot
    make px4_sitl gz_x500

-(new terminal) run the chosen script to control the drone offboard:

    cd
    cd ~/ws_offboard_control/
    ros2 run px4_ros_com offboard_control.py




Modifications to the drone's trajectory can be made in either the .py or .cpp file but only the .py was modified, the .cpp remains true to the given example in the third tutorial.
