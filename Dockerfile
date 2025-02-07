FROM osrf/ros:humble-desktop
RUN apt-get install lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null &&\
    apt-get update &&\
    apt-get install ignition-fortress -y

RUN apt install ros-humble-leo-simulator -y
RUN apt-get install -y mesa-utils libgl1-mesa-glx

RUN export LIBGL_ALWAYS_INDIRECT=1

RUN apt-get -y install python3-pip &&\
    python3 -m pip install pip --upgrade &&\
    pip3 install modern_robotics Cython


## Install Interbotix ROS packages
RUN mkdir -p /home/ws/src &&\
    cd /home/ws/src &&\
    git clone https://github.com/Interbotix/interbotix_ros_core.git -b humble --recursive &&\
    git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b humble --recursive &&\
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b humble --recursive 
RUN rm /home/ws/src/interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE &&\
    rm /home/ws/src/interbotix_ros_core/interbotix_ros_common_drivers/COLCON_IGNORE &&\
    rm /home/ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE &&\
    rm /home/ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE

## Install Leo Rover packages

RUN apt install ros-humble-leo-simulator
RUN cd /home/ws/src &&\
    git clone https://github.com/LeoRover/leo_simulator-ros2 -b humble --recursive &&\
    git clone https://github.com/LeoRover/leo_common-ros2.git -b humble --recursive


RUN export ROS_DISTRO=humble && export IGNITION_VERSION=fortress
## Build the workspace
RUN cd /home/ws &&\
    rosdep update &&\
    rosdep install --from-paths src --rosdistro $ROS_DISTRO --ignore-src -iry

RUN cd /home/ws/src && git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

RUN . /opt/ros/humble/setup.sh && cd /home/ws && colcon build --symlink-install
RUN mkdir /home/ws/src/arm_integration