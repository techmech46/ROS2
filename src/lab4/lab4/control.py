#!/usr/bin/env python3

# Import the ROS 2 Python client library
import rclpy
# Import the Node class from the ROS 2 Python client library
from rclpy.node import Node
# Import the Twist message from geometry_msgs
from geometry_msgs.msg import Twist
# Import the Odometry message from nav_msgs
from nav_msgs.msg import Odometry
# Import the LaserScan message from sensor_msgs
from sensor_msgs.msg import LaserScan
# Import the transforms3d library for quaternion to euler conversion
import transforms3d
# Import the math library for mathematical calculations
import math

# Define the GotoGoalNode class which inherits from Node
class GotoGoalNode(Node):
    # Initialize the node
    def __init__(self):
        # Call the constructor of the Node class with the node name "move_robot"
        super().__init__("move_robot")
        # Set the target x-coordinate
        self.target_x = 2
        # Set the target y-coordinate
        self.target_y = 2
        # Create a publisher for the cmd_vel topic with a queue size of 10
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Create a subscriber for the odom topic with the control_loop callback
        self.subscriber_odom = self.create_subscription(Odometry, "odom", self.control_loop, 10)
        # Create a subscriber for the laser scan topic with the obstacle_avoidance callback
        self.subscriber_laser = self.create_subscription(LaserScan, "scan", self.obstacle_avoidance, 10)
        
        # Initialize variables for obstacle detection
        self.obstacle_detected = False

    # Define the control_loop method which processes the Odometry messages
    def control_loop(self, msg):
        # Calculate the distance to the target x-coordinate
        dist_x = self.target_x - msg.pose.pose.position.x
        # Calculate the distance to the target y-coordinate
        dist_y = self.target_y - msg.pose.pose.position.y
        # Print the current position
        print('Current position: {} {}'.format(msg.pose.pose.position.x, msg.pose.pose.position.y))
        
        # Calculate the Euclidean distance to the target
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        # Print the distance to the target
        print('Distance to target: {}'.format(round(distance, 3)))
        
        # Calculate the target angle (goal_theta)
        goal_theta = math.atan2(dist_y, dist_x)
        # Extract the quaternion from the Odometry message
        quat = msg.pose.pose.orientation
        # Convert the quaternion to Euler angles
        roll, pitch, yaw = transforms3d.euler.quat2euler([quat.w, quat.x, quat.y, quat.z])
        
        # Calculate the difference between the target angle and the current yaw
        diff = math.pi - round(yaw, 2) + round(goal_theta, 2)
        # Print the current yaw
        print('Yaw: {}'.format(round(yaw, 2)))
        # Print the target angle
        print('Target angle: {}'.format(round(goal_theta, 2)))
        
        # Normalize the angle difference to the range [-pi, pi]
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        # Print the normalized angle difference
        print('Orientation difference: {}'.format(round(diff, 2)))
        
        # Create a Twist message for the velocity command
        vel = Twist()
        
        # Check for obstacle avoidance
        if self.obstacle_detected:
            vel.linear.x = 0.0
            vel.angular.z = 0.5  # Rotate in place to avoid the obstacle
            print('Obstacle detected! Rotating to avoid.')
        else:
            # If the angle difference is greater than 0.2 radians
            if abs(diff) > 0.2:
                # Stop linear movement
                vel.linear.x = 0.0
                # Set the angular velocity proportional to the angle difference
                vel.angular.z = 0.4 * round(diff, 2)
            else:
                # If the distance to the target is greater than 0.2 meters
                if abs(distance) > 0.2:
                    # Set the linear velocity proportional to the distance
                    vel.linear.x = 0.3 * round(distance, 3)
                    # Stop angular movement
                    vel.angular.z = 0.0
                else:
                    # If the robot is close to the target, stop all movement
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
        
        # Print the velocity command
        print('Speed command: {}'.format(vel))
        # Publish the velocity command
        self.publisher.publish(vel)

    # Define the obstacle_avoidance method which processes the LaserScan messages
    def obstacle_avoidance(self, msg):
        # Check for obstacles within a certain range (e.g., 0.5 meters)
        self.obstacle_detected = any(distance < 0.5 for distance in msg.ranges)

# Define the main function
def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    # Create an instance of the GotoGoalNode
    node = GotoGoalNode()
    # Spin the node to keep it running
    rclpy.spin(node)
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()

# If the script is executed directly, call the main function
if __name__ == "__main__":
    main()