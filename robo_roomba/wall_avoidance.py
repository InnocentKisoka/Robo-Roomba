#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys
from math import cos

class WallAvoidance(Node):
    def __init__(self):
        super().__init__('wall_avoidance_node')
        print("init wall_avoidance")
        # Create attributes to store odometry pose and velocity
        self.start_pose = None
        self.odom_pose = None
        self.odom_velocity = None
        self.backking_off = False

        self.state = 'APPROACHING'
        self.appraoch_min_dist = 0.2
        self.angle_error = 0.02
        self.backing_dist = 2.0
        self.inital_dist = 0.0

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
        self.left_front_tof_dist = msg.range

    def right_front_callback(self, msg):
        self.right_front_tof_dist = msg.range

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
        if self.left_front_tof_dist < 0 or self.right_front_tof_dist < 0:
            self.get_logger().info("Waiting for TOF data...")
            return
        
        if self.state == 'APPROACHING':
            self.update_approaching()
        elif self.state == 'TURNING':
            self.update_turning()
        elif self.state == 'MEASURING':
            self.update_measuring()
        elif self.state == 'FORWARD':
            self.update_forward()
        else:
            self.update_idle()

    def update_measuring(self):
        left_dist = self.left_front_tof_dist * cos(20)
        right_dist = self.right_front_tof_dist * cos(20)

        self.inital_dist = (left_dist + right_dist) / 2

        self.state = 'FORWARD'

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.00
        cmd_vel.angular.z = 0.0

        self.vel_publisher.publish(cmd_vel)

    def update_approaching(self):
        
        if (self.left_front_tof_dist < self.appraoch_min_dist or self.right_front_tof_dist < self.appraoch_min_dist):
            self.get_logger().info("APPROACHING -> TURNING")
            self.state = 'TURNING'
            return
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.15
        cmd_vel.angular.z = 0.0

        self.vel_publisher.publish(cmd_vel)
    
    def update_turning(self):

        margin_e = self.right_back_tof_dist - self.left_back_tof_dist

        self.get_logger().info("TURNING: left_back_tof_dist=%.2f, right_back_tof_dist=%.2f, margin_e=%.2f" % (self.left_back_tof_dist, self.right_back_tof_dist, margin_e))
        if (abs(margin_e) < self.angle_error and self.right_back_tof_dist < 9.0 and self.left_back_tof_dist < 9.0):
            self.get_logger().info("TURNING -> MEASURING")
            self.state = 'MEASURING'
            return
        
        linear = 0.0
        angular = 0.0
        
        if (self.left_back_tof_dist > 9.0):
            angular = 3.0
        else:
            if (margin_e < 0):
                angular = abs(margin_e)
            else:
                angular = -(abs(margin_e))
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = linear
        cmd_vel.angular.z = angular

        self.vel_publisher.publish(cmd_vel)
    
    def update_forward(self):
        if self.odom_pose is None:
            self.get_logger().info("FORWARD: Waiting for odometry data...")
            return
        
        if self.start_pose is None:
            self.start_pose = self.pose3d_to_2d(self.odom_pose)

        current_x, current_y, _ = self.pose3d_to_2d(self.odom_pose)
        start_x, start_y, _ = self.start_pose

        # Calculate the distance traveled
        distance_traveled = ((current_x - start_x) ** 2 + (current_y - start_y) ** 2) ** 0.5
        self.get_logger().info("FORWARD: Distance traveled=%.2f" % distance_traveled)
        if distance_traveled >= self.backing_dist:
            self.get_logger().info("FORWARD -> IDLE")
            self.state = 'IDLE'
            return

        # Move forward
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.15
        cmd_vel.angular.z = 0.0

        self.vel_publisher.publish(cmd_vel)

    def update_idle(self):
        self.get_logger().info("IDLE: Finished backing")
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.0
        cmd_vel.angular.z = 0.0

        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = WallAvoidance()
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
