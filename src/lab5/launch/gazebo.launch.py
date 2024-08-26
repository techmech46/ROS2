from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the URDF file
    urdf_file = '/home/kashyap/ros_ws/src/lab5/urdf/arm.urdf'

    # Return the LaunchDescription containing all processes and nodes
    return LaunchDescription([
        # Launch Gazebo with ROS 2 support
        ExecuteProcess(
            cmd=["gazebo", '--verbose', "-s", "libgazebo_ros_factory.so"],
            output="screen",
        ),

        # Node to spawn the URDF model in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            # arguments=["-entity", "lab5", "-file", urdf_file],
            arguments=["-entity", "lab5", "-b", "-file", urdf_file],
            output="screen",
        ),

        # Node to publish the robot state
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[urdf_file],
        ),
    ])
