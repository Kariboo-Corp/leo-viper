# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from ament_index_python.packages import get_package_share_directory

import xacro,os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from pathlib import Path
from launch.conditions import LaunchConfigurationEquals

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot_model_arg = LaunchConfiguration('robot_model')
    robot_ns_launch_arg = LaunchConfiguration('robot_ns')
    sim_world_launch_arg = LaunchConfiguration('sim_world')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')




    def robot_state_publisher(context):
        # Get URDF or SDF via xacro
        robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('arm_integration'),
            'config',
            f'{robot_model_arg}_controllers.yaml',
        ]
        )
        pkg_project_description = get_package_share_directory("arm_integration")
        robot_description_content = xacro.process(
        os.path.join(
            pkg_project_description,
            "urdf",
            "leo_arm_sim.urdf.xacro",
        ),
        mappings={"robot_ns": "", "hardware_type": "gz_classic"},
        )
        print(robot_description_content)

        robot_description = {'robot_description': robot_description_content, "use_sim_time": "true"}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            parameters=[
                {"robot_description": robot_description_content, "use_sim_time": True},
            ],
            executable='robot_state_publisher',
            output='both',
        )
        controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description, "use_sim_time": True},
                robot_controllers,
            ],
            output={'both': 'screen'},
        )
        return [node_robot_state_publisher, controller_manager_node]




    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'leo_viper', '-allow_renaming', 'true'],
    )

 


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{
            'use_sim_time': "true",
        }],
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
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    def launch_gz(context):
        # Set ignition resource path
        gz_resource_path_env_var = SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
                ':',
                str(Path(
                    FindPackageShare('interbotix_common_sim').perform(context)
                ).parent.resolve()),
                ':',
                str(Path(
                    FindPackageShare('interbotix_xsarm_descriptions').perform(context)
                ).parent.resolve()),
            ]
        )
        
        # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
        gz_model_uri_env_var = SetEnvironmentVariable(
            name='GAZEBO_MODEL_URI',
            value=['']
        )

        # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
        gz_model_uri_env_var = SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=['']
        )
        return [
            gz_resource_path_env_var,
            gz_model_uri_env_var,
            IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ]),
        ]),
        launch_arguments={"gz_args": sim_world_launch_arg ,"ign_args":EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='') }.items(),
        )]
    
    def xsarm_control(context):
        use_joint_pub = LaunchConfiguration('use_joint_pub')
        use_joint_pub_gui = LaunchConfiguration('use_joint_pub_gui')

        return [ IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_integration'),
                'launch',
                'xsarm_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            "use_joint_pub_gui": use_joint_pub_gui,
            "use_joint_pub" : use_joint_pub,
            'hardware_type': hardware_type_launch_arg,
            'use_sim_time': use_sim_time,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='gz_classic'
        ))
        ]
    
    ld = LaunchDescription([
        
        bridge,
        gz_spawn_entity,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[spawn_joint_state_broadcaster_node],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_joint_state_broadcaster_node,
        #         on_exit=[spawn_arm_controller_node],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_arm_controller_node,
        #         on_exit=[spawn_gripper_controller_node],
        #     )  
        # ),

        # Launch Arguments
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.'
        ),
        DeclareLaunchArgument(
            'robot_ns',
            default_value='',
            description='namespace of the robot.',
        ) ,   
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_sim'),
                'rviz',
                'xsarm_gz_classic.rviz',
            ]),
            description='file path to the config file RViz should load.',
        ),
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_common_sim'),
                'worlds',
                'interbotix.world',
            ]),
            description="the file path to the Gazebo 'world' file to load.",
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            choices=('true', 'false'),
            description='launches the Gazebo GUI if `true`.',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=('true', 'false'),
            description='launches Gazebo with verbose console logging if `true`.',
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            choices=('true', 'false'),
            description='start gzserver in debug mode using gdb.',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            choices=('true', 'false'),
            description='start Gazebo in a paused state.',
        ),
        DeclareLaunchArgument(
            'recording',
            default_value='false',
            choices=('true', 'false'),
            description='enable Gazebo state log recording.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        ),    
        DeclareLaunchArgument(
            'use_joint_pub',
            default_value='true',
            choices=('true', 'false'),
            description='launches the Joint State Publisher if set to `true`.',
        ),
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='true',
            choices=('true', 'false'),
            description='launches the Joint State Publisher GUI if set to `true`.',
        ),
        DeclareLaunchArgument(
            "sim_world",
            default_value= PathJoinSubstitution([
                FindPackageShare('leo_gz_worlds'),
                'worlds',
                'leo_empty.sdf', 
            ]),
            description="Path to the Gazebo world file",
        ),
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('arm_integration'),
                'config',
                'arm_modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        ),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        ),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    ld.add_action(OpaqueFunction(function=xsarm_control))

    ld.add_action(OpaqueFunction(function=launch_gz))
    return ld
