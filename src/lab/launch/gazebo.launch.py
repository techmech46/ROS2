import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = '/home/kashyap/ros_ws/src/lab/urdf/manipulator.urdf'

    # Path to Gazebo launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Launch description
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
            arguments=['-entity', 'manipulator', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0'],
            output='screen'
        ),

        # Node to publish the robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
    ])
