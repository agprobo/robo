#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path

import tf_transformations
import math
import time
import odrive
from odrive.enums import *
from tf2_ros import TransformBroadcaster

class OdriveMotorControl(Node):
    def __init__(self):
        super().__init__('odrive_motor_control')

        # Connect to ODrive
        self.find_odrive()
        self.odrive_setup()

        # Parameters
        self.tire_tread = 0.32  # [m] distance between wheel centers
        self.target_linear_vel = 0.0  # [m/s]
        self.target_angular_vel = 0.0  # [rad/s]
        self.tire_diameter = 0.165  # [m]
        self.encoder_cpr = 90.0  # [counts per revolution]
        self.tire_circumference = math.pi * self.tire_diameter  # [m]
        self.m_t_to_value = 1.0 / self.tire_circumference  # [turns/s]
        self.m_s_to_value = self.encoder_cpr / self.tire_circumference  # [counts/s]

        # Initial states
        self.vel_l = 0.0  # [counts/s]
        self.vel_r = 0.0  # [counts/s]
        self.new_pos_l = 0.0  # [counts]
        self.new_pos_r = 0.0  # [counts]
        self.old_pos_l = 0.0  # [counts]
        self.old_pos_r = 0.0  # [counts]
        self.x = 0.0  # [m]
        self.y = 0.0  # [m]
        self.theta = 0.0  # [rad]

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback_vel,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_path_publisher = self.create_publisher(Path, 'odom_path', 10)
        self.poses_list = []

        # Setup message
        self.odom_frame = "map"
        self.base_frame = "base_link"

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame

        # Initialize TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Start odometry and control loop
        self.create_timer(0.1, self.update)  # Update at 10 Hz

    def find_odrive(self):
        while True:
            print("Connecting to ODrive...")
            self.odrv0 = odrive.find_any()
            if self.odrv0 is not None:
                print("Connected to ODrive Successfully!")
                break
            else:
                print("Failed to connect to ODrive. Retrying...")
                time.sleep(1)  # Wait before retrying

    def odrive_setup(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("ODrive setup complete. Axis states confirmed.")

    def calcodom(self, current_time):
        # Calculate position
        self.new_pos_r = self.encoder_cpr * self.odrv0.axis0.encoder.pos_estimate
        self.new_pos_l = self.encoder_cpr * self.odrv0.axis1.encoder.pos_estimate

        delta_pos_r = self.new_pos_r - self.old_pos_r
        delta_pos_l = self.new_pos_l - self.old_pos_l
        
        self.old_pos_r = self.new_pos_r
        self.old_pos_l = self.new_pos_l

        # Handle encoder overflow
        half_cpr = self.encoder_cpr / 2.0
        if delta_pos_r > half_cpr: 
            delta_pos_r -= self.encoder_cpr
        elif delta_pos_r < -half_cpr: 
            delta_pos_r += self.encoder_cpr
        if delta_pos_l > half_cpr: 
            delta_pos_l -= self.encoder_cpr
        elif delta_pos_l < -half_cpr: 
            delta_pos_l += self.encoder_cpr
        
        # Convert counts to meters
        delta_pos_r_m = delta_pos_r / self.m_s_to_value
        delta_pos_l_m = delta_pos_l / self.m_s_to_value

        # Compute distance and angle
        d = (delta_pos_r_m + delta_pos_l_m) / 2.0
        th = (delta_pos_r_m - delta_pos_l_m) / self.tire_tread
        
        xd = math.cos(th) * d
        yd = -math.sin(th) * d

        # Update pose
        self.x += math.cos(self.theta) * xd - math.sin(self.theta) * yd
        self.y += math.sin(self.theta) * xd + math.cos(self.theta) * yd
        self.theta = (self.theta + th) % (2 * math.pi)

        # Compute velocity
        self.vel_r = self.encoder_cpr * self.odrv0.axis0.encoder.vel_estimate
        self.vel_l = self.encoder_cpr * self.odrv0.axis1.encoder.vel_estimate
        v = self.tire_circumference * (self.vel_r + self.vel_l) / (2.0 * self.encoder_cpr)
        w = self.tire_circumference * (self.vel_r - self.vel_l) / (self.tire_tread * self.encoder_cpr)

        # Publish odometry
        self.odom_msg.header.stamp = current_time.to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x = v
        self.odom_msg.twist.twist.angular.z = w
        self.odom_publisher.publish(self.odom_msg)

        # Broadcast transforms
        self.broadcast_transform((0.0, 0.0, 0.0), tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0), current_time, "odom", "map")
        self.broadcast_transform((self.x, self.y, 0.0), q, current_time, "base_link", "odom")
        # Debug logging
        self.get_logger().info(f"Publishing Odometry: x={self.x}, y={self.y}, theta={self.theta}, v={v}, w={w}")


        # Publish path
        temp_pose = PoseStamped()
        temp_pose.header.stamp = current_time.to_msg()
        temp_pose.header.frame_id = "map"
        temp_pose.pose.position.x = self.x
        temp_pose.pose.position.y = self.y
        temp_pose.pose.orientation.z = q[2]
        temp_pose.pose.orientation.w = q[3]
        self.poses_list.append(temp_pose)

        path = Path()
        path.header.stamp = current_time.to_msg()
        path.header.frame_id = "map"
        path.poses = self.poses_list
        self.odom_path_publisher.publish(path)

    def odrive_control(self):
        rate = self.create_rate(10)
        
        print('Starting ODrive control loop')
        while rclpy.ok():
            try:
                current_time = self.get_clock().now()
                right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)
                self.get_logger().info(f"Setting velocities: right = {right_vel}, left = {left_vel}")

                self.odrv0.axis0.controller.input_vel = right_vel
                self.odrv0.axis1.controller.input_vel = -left_vel

                self.calcodom(current_time)
                rate.sleep()
            except Exception as e:
                self.get_logger().error(f"Exception: {e}")
                break

    def calc_relative_vel(self, target_linear_vel, target_angular_vel):
        # Convert [m/s] to [turn/s] (rotation speed)
        right_vel = (target_linear_vel + (target_angular_vel * self.tire_tread / 2.0)) * self.m_t_to_value
        left_vel = (target_linear_vel - (target_angular_vel * self.tire_tread / 2.0)) * self.m_t_to_value
        return right_vel, left_vel

    def callback_vel(self, twist_msg):
        # Save target velocity
        self.get_logger().info(f"Received velocity command: linear.x = {twist_msg.linear.x}, angular.z = {twist_msg.angular.z}")
        self.target_linear_vel = twist_msg.linear.x  # [m/s]
        self.target_angular_vel = twist_msg.angular.z  # [rad/s]

    def broadcast_transform(self, translation, rotation, timestamp, child_frame, parent_frame):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(t)

    def update(self):
        current_time = self.get_clock().now()
        self.odrive_control()
        self.calcodom(current_time)

def main(args=None):
    rclpy.init(args=args)
    try:
        odrive_motor_control = OdriveMotorControl()
        rclpy.spin(odrive_motor_control)
    except KeyboardInterrupt:
        pass
    finally:
        odrive_motor_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
