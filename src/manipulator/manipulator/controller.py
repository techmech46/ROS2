#!/usr/bin/env python3

# colcon build --packages-select urdf_tutorial
# ros2 run urdf_tutorial controller --ros-args -p end_location:=[3.5,1.5,-1.2]

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    
    def __init__(self):
        super().__init__('trajectory_node')
        topic_ = "/joint_trajectory_controller/joint_trajectory"
        
        # List of joint names
        self.joints = ['base_arm1_joint', 'arm1_arm2_joint', 'arm2_arm3_joint']
        
        # Declare parameter for joint angles
        self.declare_parameter("joint_angles", [0.5, 0.5, 0.2])
        
        # Retrieve parameter value
        self.goal_ = self.get_parameter("joint_angles").value
        
        # Publisher for JointTrajectory messages
        self.publisher_ = self.create_publisher(JointTrajectory, topic_, 10)
        
        # Timer to periodically call the callback
        self.timer_ = self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        # Create and populate JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = self.joints
        
        point = JointTrajectoryPoint()
        point.positions = self.goal_
        point.time_from_start = Duration(sec=2)
        
        msg.points.append(point)
        
        # Publish the JointTrajectory message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = TrajectoryPublisher()
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
