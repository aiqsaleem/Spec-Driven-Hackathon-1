import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    urdf_model_path = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'robot.urdf')

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(urdf_model_path).read()}
        ]
    )

    # Joint State Publisher node (for GUI control)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(get_package_share_directory('my_robot_bringup'), 'config', 'urdf_control_config.yaml')
        ],
        output='both'
    )

    # Joint Trajectory Controller Spawner
    joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    # Return launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_trajectory_spawner)

    return ld