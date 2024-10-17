#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtleBotController(Node):
    def __init__(self):

        # Initialize the node with the name 'turtlebot_controller'
        super().__init__('turtlebot3_controller')
        # Define a publisher to control robot velocity
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Define a subscriber to listen to LaserScan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # Initialize the velocity message
        self.vel_msg = Twist()
        #threshold parameter
        self.declare_parameter("threshold", 0.5)   
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        #maximum linear velocity parameter
        self.declare_parameter("max_lin_vel", 0.22)   
        self.max_lin_vel = self.get_parameter("max_lin_vel").get_parameter_value().double_value
        #maximum angular velocity parameter
        self.declare_parameter("max_ang_vel", 1.5)   
        self.max_ang_vel = self.get_parameter("max_ang_vel").get_parameter_value().double_value
        # Control loop frequency parameter
        self.declare_parameter("frequency", 10.0)   
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        #timer
        self.timer = self.create_timer( 1.0/self.frequency, self.control_loop)
        # Initialize distance attributes as to infinity
        self.min_front_distance = float('inf')  # Default value (no obstacle in front)
        self.min_left_distance = float('inf')   # Default value (no obstacle on the left)
        self.min_right_distance = float('inf')  # Default value (no obstacle on the right)
        # Flag to indicate whether to move forward or stop and rotate
        self.state = 1 # Possible states: 1 (move forward), 2 (rotate)
        # Define the number of rays to check for obstacles
        self.front_angle_range = 20  # Check the first and last 10 rays
        self.side_angle_range = 30   # Check the range that goes from 60 to 120 degrees to the left and right

    def lidar_callback(self, data):
        # Get the total number of /scan readings
        num_readings = len(data.ranges) #it is an array in which every entry represent the value of a specific angle
        #the initial value is the value at the starting point of the sensor that in this case is the direction of movement
        #therefore to check for frontal obstacle we consider the first n and last n entries of the array

        # Get front distances (first and last part of the scan array)
        front_distances = data.ranges[:self.front_angle_range] + data.ranges[-self.front_angle_range:] #we consider a range of 20 degree
        #on the right and 20 degree on the left
        front_distances = [distance for distance in front_distances if not float('inf') == distance] #if the object is not detected
        #the value returned is infinite, so we remove this value to prevent errors
        self.min_front_distance = min(front_distances) if front_distances else float('inf') #if the list is empty (object is too far), then we set infinite
        # Get left and right distances
        #I scan the section that correspond to 90 degrees to the left and 90 degrees to the right
        #I do this performing integer division (split the dataset into 4 part) and than remove/add the range desired
        left_distances = data.ranges[num_readings // 4 - self.side_angle_range:num_readings // 4 + self.side_angle_range]
        right_distances = data.ranges[3 * num_readings // 4 - self.side_angle_range:3 * num_readings // 4 + self.side_angle_range]
        # Clean up infinite distances as in the previous case
        self.min_left_distance = min([d for d in left_distances if d != float('inf')], default=float('inf'))
        self.min_right_distance = min([d for d in right_distances if d != float('inf')], default=float('inf'))

    def control_loop(self):
        # Case 1: move forward
        if self.state == 1:
            if self.min_front_distance < self.threshold:
                # Obstacle detected in front, stop the robot and switch to rotating state
                self.get_logger().info(f"Obstacle detected at {self.min_front_distance:.2f} meters! Stopping the robot and rotating.")
                self.vel_msg.linear.x = 0.0  # Stop forward movement
                self.state = 2
            else:
                # No obstacle, move forward
                self.vel_msg.linear.x = self.max_lin_vel # Move forward with a constant speed
                self.vel_msg.angular.z = 0.0  # No rotation
        # Case 2: rotate until it finds a suitable direction
        elif self.state == 2:
            if self.min_front_distance < self.threshold:
                # Still obstacle in front, check left and right distances
                if self.min_left_distance > self.min_right_distance:
                    # More space on the left, rotate left
                    self.get_logger().info("Rotating left to find a free direction.")
                    self.vel_msg.angular.z = self.max_ang_vel # Rotate left
                else:
                    # More space on the right, rotate right
                    self.get_logger().info("Rotating right to find a free direction.")
                    self.vel_msg.angular.z = -self.max_ang_vel  # Rotate right
                # Set linear velocity to zero to prevent backward movement
                self.vel_msg.linear.x = 0.0
            else:
                # Free direction found, switch back to moving forward
                self.get_logger().info("Free direction found! Resuming forward movement.")
                self.state = 1 #reset the state to the moving forward situation
                self.vel_msg.angular.z = 0.0  # Stop rotating
                self.vel_msg.linear.x = self.max_lin_vel  # Start moving forward again
        # Publish the velocity command
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create the controller node
    controller = TurtleBotController()

    # Spin the node so it keeps running
    rclpy.spin(controller)

    # Clean up and shut down the node
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
