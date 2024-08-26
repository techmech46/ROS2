#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.clock import Clock
from math import radians, cos, sin, atan, degrees, sqrt, acos
import sys

class TrajectoryPublisher(Node):
    
    def __init__(self):
        super().__init__('trajectory_node')
        topic_ = "/joint_states"
        self.joints = ['base_arm1_joint', 'arm1_arm2_joint', 'arm2_arm3_joint']
        self.l1 = 0.5
        self.l2 = 0.5
        self.l3 = 0.3
        self.theta1 = None
        self.theta2 = None
        self.theta3 = None
        self.x = float(sys.argv[2])
        self.z = float(sys.argv[1])
        self.fi = float(sys.argv[3])
        self.goal_ = None
        self.ik()
        self.publisher_ = self.create_publisher(JointState, topic_, 10)
        self.timer_ = self.create_timer(0.5,self.timer_callback)

    def ik(self):
        p2_x = self.x - self.l3 * cos(radians(self.fi))
        p2_z = self.z - self.l3 * sin(radians(self.fi))
        N1 = p2_x * p2_x + p2_z * p2_z
        N2 = self.l1 * self.l1 + self.l2 * self.l2
        D = 2 * self.l1 * self.l2
        ans = (N1 - N2) / D
        # print(ans)
        try:
            self.theta2 = acos(ans)
        except:
            self.get_logger().error("Coordinates are out of workspace!!!")
            exit()
        b = atan((self.l2 * sin(self.theta2)) / (self.l2 * cos(self.theta2) + self.l1))
        a = atan(p2_z / p2_x)
        self.theta1 = a - b
        self.theta3 = radians(self.fi) - (self.theta1 + self.theta2)
        self.goal_  = [round(self.theta1,2), round(self.theta2,2), round(self.theta3,2)]
        self.get_logger().info("IK Results...")
        self.get_logger().info(f"Joint angle: {self.goal_}")
        
    def timer_callback(self):
        msg = JointState()
        current_time = Clock().now().to_msg()
        msg.header.stamp.sec = current_time.sec
        msg.header.stamp.nanosec = current_time.nanosec
        msg.name = self.joints
        msg.position = self.goal_
        self.publisher_.publish(msg)
        self.get_logger().info("Reached the position...")
        exit()
        
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()