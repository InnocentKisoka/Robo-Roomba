#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu

import sys

class RoboRoombaControllerV2(Node):
    def __init__(self):
        super().__init__('robo_roomba_controller_v2_node')
        print("init robo_roomba_controller")
        # Create attributes to store odometry pose and velocity
        self.start_pose = None
        self.odom_pose = None
        self.odom_velocity = None
        self.backking_off = False
        self.imu_linear_acceleration = None

        self.state = 'APPROACHING'
        self.appraoch_min_dist = 0.15
        self.turning_min_dist = 0.30

        self.angle_error = 0.02
        self.angle_degrees = 30.0
        self.backing_dist = 2.0

        self.alternate_flag = False
        self.alternate = -1
        self.wiggle_count = 25

        self.approaching_alteranate = 1

        self.no_movement_count = 0
        self.no_movement_threshold = 20

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
        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 10)


        
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.get_logger().info('Wall Detection Node started...')
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
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

    def imu_callback(self, msg):
        self.imu_linear_acceleration = msg.linear_acceleration
        
    
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
        else:
            self.update_idle()


    def update_approaching(self):
       
        if (self.detect_front_collision()):
            self.get_logger().info("APPROACHING: Wall detected, switching to TURNING state")
            self.state = 'TURNING'
            return
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.15

        angular = 0.0
        if self.approaching_alteranate < self.wiggle_count:
            self.approaching_alteranate += 1
            angular = 3.0
        elif self.approaching_alteranate < 2*self.wiggle_count:
            self.approaching_alteranate += 1
            angular = -2.7
        else:
            self.approaching_alteranate = 0
        
        cmd_vel.angular.z = angular

        self.vel_publisher.publish(cmd_vel)

    

    def detect_front_collision(self):
        if self.imu_linear_acceleration is None:
            return False
        
        if (self.left_front_tof_dist < self.appraoch_min_dist or self.right_front_tof_dist < self.appraoch_min_dist):
            self.get_logger().info("APPROACHING: Front sensor detected wall")
            self.no_movement_count = 0
            return True
        
        
        # self.get_logger().info("APPROACHING: imu_linear_acceleration: x=%.2f" % (self.imu_linear_acceleration.x))
        if self.imu_linear_acceleration.x > -0.18:
            self.no_movement_count = 0
        else:
            self.no_movement_count += 1
            if self.no_movement_count > self.no_movement_threshold:
                self.get_logger().info("APPROACHING: no movement count exceeded")
                self.no_movement_count = 0
                return True
        return False

    def detect_back_collision(self):
        if self.imu_linear_acceleration is None:
            return False
        
        if (self.left_back_tof_dist < self.turning_min_dist or self.right_back_tof_dist < self.turning_min_dist):
            self.get_logger().info("Front sensor detected wall")
            self.no_movement_count = 0
            return True
        
        
        self.get_logger().info("imu_linear_acceleration: x=%.2f" % (self.imu_linear_acceleration.x))
        if self.imu_linear_acceleration.x < -.12:
            self.no_movement_count = 0
        else:
            self.no_movement_count += 1
            if self.no_movement_count > self.no_movement_threshold:
                self.get_logger().info("no movement count exceeded")
                self.no_movement_count = 0
                return True
        return False

    def update_turning(self):
        # self.get_logger().info("TURNING: left_back_tof_dist=%.2f, right_back_tof_dist=%.2f" % (self.left_back_tof_dist, self.right_back_tof_dist))
        if (self.detect_back_collision()):
            self.start_pose = None
            self.get_logger().info("TURNING: swithcing to APPROACHING state")
            self.state = 'APPROACHING'
            self.alternate = -1 * self.alternate
            return

        linear = -0.05
        angular = self.alternate *  1.5
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = linear
        cmd_vel.angular.z = angular

        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = RoboRoombaControllerV2()
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
