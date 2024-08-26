from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the URDF file
    urdf_file = '/home/kashyap/ros_ws/src/lab5/urdf/arm.urdf'

    # Node to publish joint states using a GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{'robot_description': urdf_file}]
        # arguments=[urdf_file]
    )

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=[urdf_file]
    )

    # Node to run RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    # List of nodes to launch
    nodes_to_run = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]

    # Return the LaunchDescription containing all nodes
    return LaunchDescription(nodes_to_run)
