#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'trajectory_node'
        super().__init__('trajectory_node')

        # Topic for the joint trajectory controller
        topic_ = "/joint_trajectory_controller/joint_trajectory"

        # List of joints to control
        self.joints = ['base_arm1_joint', 'arm1_arm2_joint', 'arm2_arm3_joint']

        # Goal positions for the joints
        self.goal_ = [0.0, 0.0, 0.75]

        # Optionally, you can set the goal from parameters
        # self.declare_parameter("joint_angles", [1.5, 0.5, 1.2])
        # self.goal_ = self.get_parameter("joint_angles").value

        # Publisher for the JointTrajectory message
        self.publisher_ = self.create_publisher(JointTrajectory, topic_, 10)

        # Timer to periodically call the timer_callback function
        self.timer_ = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # Create a JointTrajectory message
        msg = JointTrajectory()

        # Set the joint names in the message
        msg.joint_names = self.joints

        # Create a JointTrajectoryPoint and set the goal positions
        point = JointTrajectoryPoint()
        point.positions = self.goal_

        # Set the time from the start for the trajectory point
        point.time_from_start = Duration(sec=2)

        # Append the point to the message
        msg.points.append(point)

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the TrajectoryPublisher node
    node = TrajectoryPublisher()

    # Keep the node running, handling callbacks
    rclpy.spin(node)

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
