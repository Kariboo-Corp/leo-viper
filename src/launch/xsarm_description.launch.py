# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from launch.conditions import IfCondition
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration):
    pkg_project_description = get_package_share_directory("arm_integration")
    robot_ns = context.perform_substitution(namespace)
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    use_joint_pub_launch_arg = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )
    print(f"Robot description namespace {robot_ns}")
    print(f"Hardware type {hardware_type_launch_arg.perform(context)}")

    if robot_ns == "":
        robot_gazebo_name = "mobile_px100"
        node_name_prefix = ""
    else:
        robot_gazebo_name = "mobile_px100_" + robot_ns
        node_name_prefix = robot_ns + "_"
    robot_desc = xacro.process(
        os.path.join(
            pkg_project_description,
            "urdf",
            "leo_arm_sim.urdf.xacro",
        ),
        mappings={"robot_ns": robot_ns, "hardware_type": hardware_type_launch_arg.perform(context)},
    )
    print(robot_desc)

    # Launch robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"robot_description": robot_desc},
        ],
    )
    # # Spawn a robot inside a simulation
    # leo_rover = Node(
    #     namespace=robot_ns,
    #     package="ros_gz_sim",
    #     executable="create",
    #     name="ros_gz_sim_create",
    #     output="both",
    #     arguments=[
    #         "-topic",
    #         "robot_description",
    #         "-name",
    #         robot_gazebo_name,
    #         "-z",
    #         "1.65",
    #     ],
    # )
    

    print(robot_name_launch_arg.perform(context))
    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub_launch_arg),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_ns,
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
        output={'both': 'log'},
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster',
        ],
        parameters=[{
            'use_sim_time': "true",
        }],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller',
        ],
        parameters=[{
            'use_sim_time': "true",
        }]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'gripper_controller',
        ],
        parameters=[{
            'use_sim_time': "true",
        }]
    )


    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui_launch_arg),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_ns,
        output={'both': 'log'},
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_ns,
        arguments=[
            '-d', rvizconfig_launch_arg,
            '-f', "base_link",

        ],
        parameters=[{
            'use_sim_time': use_sim_time_param

        }],
        output={'both': 'log'},
    )
    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        arguments=[
            robot_ns + "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            robot_ns + "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            robot_ns + "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            robot_ns + "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            robot_ns
            + "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            robot_ns + "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            robot_ns + '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Camera image bridge
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name=node_name_prefix + "image_bridge",
        arguments=[robot_ns + "/camera/image_raw"],
        output="screen",
    )
    return [
        robot_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[spawn_joint_state_broadcaster_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster_node,
                on_exit=[spawn_arm_controller_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_arm_controller_node,
                on_exit=[spawn_gripper_controller_node],
            )  
        ),

        # leo_rover,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        topic_bridge,
        image_bridge,
    ]
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
def generate_launch_description():
    declared_arguments = []


    namespace = LaunchConfiguration("robot_ns")


    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ns',
            default_value='',
            description='namespace of the robot.',
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub',
            default_value='true',
            choices=('true', 'false'),
            description='launches the joint_state_publisher node.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='true',
            choices=('true', 'false'),
            description='launches the joint_state_publisher GUI.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_descriptions'),
                'rviz',
                'xsarm_description.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(),
    )

    return LaunchDescription(
        declared_arguments+[ OpaqueFunction(function=spawn_robot, args=[namespace])]
    )
