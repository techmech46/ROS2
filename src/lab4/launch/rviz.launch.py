from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf = '/home/kashyap/ros_ws/src/lab4/urdf/three_wheeled_robot.urdf'
    # Uncomment and set the path to your RViz configuration file if you have one
    # rviz_config_file = os.path.join(package_dir, 'config.rviz')

    return LaunchDescription([
        # Node to publish the state of the robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        ),
        # Node to publish the joint states using a GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]
        ),
        # Node to launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # Uncomment and set the argument if using an RViz configuration file
            # arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
