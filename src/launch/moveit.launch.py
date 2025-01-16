import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_interbotix_moveit = get_package_share_directory("interbotix_xsarm_moveit")
    pkg_interbotix_control = get_package_share_directory("interbotix_xsarm_control")
    pkg_project_description = get_package_share_directory("arm_integration")

    robot_desc = xacro.process(
        os.path.join(
            pkg_project_description,
            "urdf",
            "leo_arm_sim.urdf.xacro",
        ),
        mappings={"robot_ns": ""},
    )

    print(robot_desc)

    spawn_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interbotix_control, "launch", "xsarm_control.launch.py")
        ),
        launch_arguments={
            "robot_model":"mobile_px100",
            "robot_description": robot_desc,
            "mode_config": os.path.join(pkg_interbotix_control, "config", "arm_modes.yaml"), 
            "use_sim":"true"
            }.items(),
    )


    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interbotix_moveit, "launch", "xsarm_moveit.launch.py")
        ),
        launch_arguments={
                "robot_model":"mobile_px100", 
                "robot_description": robot_desc,
                "hardware_type":"fake"
            }.items(),
    )

    return LaunchDescription([spawn_control,spawn_robot])