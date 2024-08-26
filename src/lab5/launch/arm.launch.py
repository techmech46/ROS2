import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Paths to the URDF and controller configuration files
    urdf_file = '/home/kashyap/ros_ws/src/lab5/urdf/arm.urdf'
    # controller_file = '/home/kashyap/ros_ws/src/lab5/config/control.yaml'

    # Parse the URDF file using xacro
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/kashyap/ros_ws/src/lab5/launch/gazebo.launch.py'),
    )

    # Node to publish the robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-entity", "lab5", "-b", "-file", urdf_file],
        output='screen'
    )

    # Process to load the joint state controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Process to load the joint trajectory controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    # Event handler to load joint state controller after the entity is spawned
    event_handler_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    # Event handler to load joint trajectory controller after the joint state controller is loaded
    event_handler_joint_trajectory = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    # # Nodes to manage the controllers using spawner
    # spawner_joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # spawner_joint_trajectory_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    # )

    # LaunchDescription that includes all nodes and processes
    return LaunchDescription([
        # Launch Gazebo with ROS 2 support
        ExecuteProcess(
            cmd=["gazebo", '--verbose', "-s", "libgazebo_ros_factory.so"],
            output="screen",
        ),

        # Include Gazebo, robot state publisher, and spawn entity nodes
        gazebo,
        node_robot_state_publisher,
        spawn_entity,

        # Event handlers
        event_handler_joint_state,
        event_handler_joint_trajectory,

        # Nodes to manage controllers
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            # parameters=[params, controller_file],
            output="screen"
        ),
    #     spawner_joint_state_broadcaster,
    #     spawner_joint_trajectory_controller,
    ])
