#!/usr/bin/env python3


#colcon build --packages-select lab5 –symlink-install
#ros2 run lab5 control --ros-args -p end_location:=[0.15,0.5,-0.2]

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    
    def __init__(self):
        super().__init__('trajectory_node')
        topic_ = "/joint_trajectory_controller/joint_trajectory"
        self.joints = ['base_arm1_joint', 'arm1_arm2_joint', 'arm2_arm3_joint']
        self.goal_ =[0.0, 0.75, 0.75]
        print(self.goal_)
        #self.declare_parameter("joint_angles", [1.5, 0.5, 1.2])
        #self.goal_=self.get_parameter("joint_angles").value
        self.publisher_ = self.create_publisher(JointTrajectory, topic_, 10)
        self.timer_ = self.create_timer(1,self.timer_callback)

        
        
    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()