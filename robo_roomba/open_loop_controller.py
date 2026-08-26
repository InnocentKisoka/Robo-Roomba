#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller_node')
        print("init OpenLoopController")
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        print("create publisher")
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        print("create subscriber")
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.get_logger().info('Node started...')
        self.timer = self.create_timer(0.1, self.update_callback)
        self.time_elapsed = 0.0
        self.cycle_duration = 12.5
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        print("odom_callback triggered")
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
    
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
        cmd_vel = Twist() 
        linear = 0.2        
        angular = 0.5          
        t = self.time_elapsed

        self.get_logger().info(
            "update_callback: t=%.2f, linear=%.2f, angular=%.2f" % (t, linear, angular),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

        cmd_vel.linear.x = linear
        if int(t // self.cycle_duration) % 2 == 0:
            cmd_vel.angular.z = angular
        else:
            cmd_vel.angular.z = -angular

        self.get_logger().info(
            "update_callback: t=%.2f, linear=%.2f, angular=%.2f" % (t, cmd_vel.linear.x, cmd_vel.angular.z),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        self.time_elapsed += 0.1
        
        # Publish the command
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = OpenLoopController()
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
