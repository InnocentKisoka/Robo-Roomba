#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys

class WallDetection(Node):
    def __init__(self):
        super().__init__('wall_detection_node')
        print("init wall_detection")
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        self.left_front_tof_dist = -10
        self.right_front_tof_dist = -10
        self.left_back_tof_dist = -10
        self.right_back_tof_dist = -10
                
        # Create a publisher for the topic 'cmd_vel'
        print("create publisher")
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        print("create subscriber")
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.left_front_tof_subscriber = self.create_subscription(Range, 'range_3', self.left_front_callback, 10)
        self.right_front_tof_subscriber = self.create_subscription(Range, 'range_1', self.right_front_callback, 10)
        self.left_back_tof_subscriber = self.create_subscription(Range, 'range_2', self.left_back_callback, 10)
        self.right_back_tof_subscriber = self.create_subscription(Range, 'range_0', self.right_back_callback, 10)


        
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.get_logger().info('Wall Detection Node started...')
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        print("odom_callback triggered")
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist

    def left_front_callback(self, msg):
        self.left_tof_dist = msg.range

    def right_front_callback(self, msg):
        self.right_tof_dist = msg.range

    def left_back_callback(self, msg):
        self.left_back_tof_dist = msg.range

    def right_back_callback(self, msg):
        self.right_back_tof_dist = msg.range
        
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def update_callback(self):
        if self.left_tof_dist < 0 or self.right_tof_dist < 0:
            self.get_logger().info("Waiting for TOF data...")
            return
        linear = 0.0        
        angular = 0.0

        margin_e = self.right_tof_dist - self.left_tof_dist

        min_dist = 0.1
        angle_error = 0.01
        if (self.left_tof_dist > min_dist and self.right_tof_dist > min_dist):
            linear = 0.2
            angular = 0.0
        elif (abs(margin_e) < angle_error):
            linear = 0.0
            angular = 0.0
        else:
            linear = 0.0
            if (margin_e < 0):
                angular = -0.1
            else:
                angular = 0.1
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = linear
        cmd_vel.angular.z = angular

        self.get_logger().info(
            "update_callback: linear=%.2f, angular=%.2f, margin_e=%.2f" % (linear, angular, margin_e),
            throttle_duration_sec=0.5
        )
        
        # Publish the command
        self.vel_publisher.publish(cmd_vel)



def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = WallDetection()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the RoboMaster is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
