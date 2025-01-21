# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="robotrainer3_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="demo_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robotrainer3_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="demo.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo_classic",
            default_value="false",
            description="Enable gazebo classic simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Enable gazebo simulation.",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    sim_gazebo_classic = LaunchConfiguration("sim_gazebo_classic")
    sim_gazebo = LaunchConfiguration("sim_gazebo")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robotrainer3.rviz"]
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " prefix:=",
            prefix,
            " use_mock_hardware:=",
            use_mock_hardware,
            " mock_sensor_commands:=",
            mock_sensor_commands,
            " sim_gazebo_classic:=",
            sim_gazebo_classic,
            " sim_gazebo:=",
            sim_gazebo,
            " simulation_controllers:=",
            robot_controllers,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Gazebo for Humble
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_ign_gazebo"), "/launch", "/ign_gazebo.launch.py"]
        ),
        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
        condition=IfCondition(sim_gazebo),
    )

    # Gazebo for Jazzy
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     ),
    #     launch_arguments={"gz_args": " -r -v 4 empty.sdf"}.items(),
    #     condition=IfCondition(sim_gazebo),
    # )

    # Classic Gazebo
    gazebo_classic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        condition=IfCondition(sim_gazebo_classic),
    )

    # Spawn robot (Humble)
    # gazebo_spawn_robot = Node(
    #     package="ros_ign_gazebo",
    #     executable="create",
    #     name="spawn_demo",
    #     arguments=["-name", "demo", "-topic", "robot_description"],
    #     output="screen",
    #     condition=IfCondition(sim_gazebo),
    # )

    # Spawn robot (Jazzy)
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "demo",
            "-allow_renaming",
            "true",
        ],
        condition=IfCondition(sim_gazebo),
    )

    # Spawn robot (Gazebo Classic)
    gazebo_classic_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_demo",
        arguments=["-entity", "demo", "-topic", "robot_description"],
        output="screen",
        condition=IfCondition(sim_gazebo_classic),
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    robot_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             robot_controller],
        output='screen'
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of gazebo_spawn_robot
    delay_joint_state_broadcaster_spawner_after_gazebo_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_joint_state_broadcaster_spawner_after_gazebo_classic_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_classic_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay rviz start after robot_controller_spawner to avoid unnecessary warning output.
    delay_rviz_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[TimerAction(
                period=1.0,
                actions=[rviz_node],
            ),],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo,
            gazebo_classic,
            gazebo_spawn_robot,
            gazebo_classic_spawn_robot,
            # control_node,
            robot_state_pub_node,
            delay_joint_state_broadcaster_spawner_after_gazebo_spawn_robot,
            delay_joint_state_broadcaster_spawner_after_gazebo_classic_spawn_robot,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_rviz_after_robot_controller_spawner,
        ]
    )
