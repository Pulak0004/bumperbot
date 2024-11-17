#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius is = %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation is = %f" % self.wheel_separation_)

        self.right_wheel_pre_pos_ = 0.0
        self.left_wheel_pre_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0 
        self.y_ = 0.0
        self.theta_ = 0.0

        self.joint_subscriber_ = self.create_subscription(JointState, "joint_states", self.jointStateCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.transform_broadcaster_ = TransformBroadcaster(self)
        self.transform_stampd_ = TransformStamped()
        self.transform_stampd_.header.frame_id = "odom"
        self.transform_stampd_.child_frame_id = "base_footprint_noisy"

    def jointStateCallback(self, msg):
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.003)
        wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.003)

        dp_left = wheel_encoder_left - self.left_wheel_pre_pos_
        dp_right = wheel_encoder_right - self.right_wheel_pre_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.right_wheel_pre_pos_ = msg.position[0]
        self.left_wheel_pre_pos_ = msg.position[1]
        self.prev_time_ = Time.from_msg(msg.header.stamp)
        
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear_velocity = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular_velocity = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        d_s = (dp_left + dp_right) * self.wheel_radius_ / 2
        d_theta = (dp_right - dp_left) * self.wheel_radius_ / self.wheel_separation_ 

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear_velocity
        self.odom_msg_.twist.twist.angular.z = angular_velocity

        self.transform_stampd_.transform.translation.x = self.x_
        self.transform_stampd_.transform.translation.y = self.y_
        self.transform_stampd_.transform.rotation.x =  q[0]
        self.transform_stampd_.transform.rotation.y =  q[1]
        self.transform_stampd_.transform.rotation.z =  q[2]
        self.transform_stampd_.transform.rotation.w =  q[3]
        self.transform_stampd_.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub_.publish(self.odom_msg_)
        self.transform_broadcaster_.sendTransform(self.transform_stampd_)


def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()    







        