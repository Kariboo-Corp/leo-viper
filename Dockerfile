# Base ROS image
FROM osrf/ros:humble-desktop

# Configure shell to use bash
SHELL ["/bin/bash", "-c"]

# Mise à jour et installation de base pour éviter les erreurs d'apt
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils gnupg curl lsb-release

ENV ROS_DOMAIN_ID=30
RUN export LIBGL_ALWAYS_INDIRECT=1

# Installation de Gazebo et Ignition Fortress
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y ignition-fortress

# Installation de packages ROS supplémentaires (Leo Simulator)
RUN apt-get install -y ros-humble-leo-simulator mesa-utils libgl1-mesa-glx

# Installation de Python pip et dépendances
RUN apt-get install -y python3-pip && \
    python3 -m pip install --upgrade pip && \
    pip3 install modern_robotics Cython

# Installation des packages Interbotix ROS
RUN mkdir -p /home/ws/src && cd /home/ws/src && \
    git clone https://github.com/Interbotix/interbotix_ros_core.git -b humble --recursive && \
    git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b humble --recursive && \
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b humble --recursive && \
    # Suppression des fichiers COLCON_IGNORE
    rm interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE && \
    rm interbotix_ros_core/interbotix_ros_common_drivers/COLCON_IGNORE && \
    rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE && \
    rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE

# Installation des packages Leo Rover ROS2
RUN cd /home/ws/src && \
    git clone https://github.com/LeoRover/leo_simulator-ros2.git -b humble --recursive && \
    git clone https://github.com/LeoRover/leo_common-ros2.git -b humble --recursive

# Préparation du workspace (sans build colcon)
RUN rosdep update && rosdep install --from-paths /home/ws/src -iry

# Création du répertoire d'intégration du bras (vide pour l'instant)
RUN mkdir -p /home/ws/src/arm_integration

# Ajout d'un script d'environnement pour travailler plus facilement
RUN echo 'source /opt/ros/humble/setup.bash && cd /home/ws' > /root/start_env.sh && \
    chmod +x /root/start_env.sh

# Définir le répertoire de travail par défaut
WORKDIR /home/ws

# Construire le workspace avec colcon
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install