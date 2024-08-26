import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = '/home/kashyap/ros_ws/src/lab/urdf/manipulator.urdf'
    controller_file = '/home/kashyap/ros_ws/src/lab/config/control.yaml'
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    return LaunchDescription([
        # Start Gazebo with the factory plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Node to spawn the URDF model into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'manipulator', '-file', urdf_file],
            output='screen'
        ),

        # Node to publish the robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),

        # Node for ros2_control
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_file],
            output='screen'
        ),

        # Load controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[controller_file],
                    output='screen'
                ),
                on_exit=[
                    ExecuteProcess(
                        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'joint_state_broadcaster'],
                        output='screen'
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'joint_trajectory_controller'],
                        output='screen'
                    )
                ]
            )
        )
    ])
