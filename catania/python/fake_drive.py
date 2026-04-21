#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class FakeDrive(Node):
    def __init__(self):
        super().__init__('fake_drive')

        # Parameters
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.23)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 30.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        # Wheel joint positions
        self.left_pos = 0.0
        self.right_pos = 0.0

        self.left_front_joint = 'left_front_wheel_joint'
        self.left_rear_joint = 'left_rear_wheel_joint'
        self.right_front_joint = 'right_front_wheel_joint'
        self.right_rear_joint = 'right_rear_wheel_joint'

        # ROS interfaces
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.update)

        self.last_time = self.get_clock().now()

        self.get_logger().info('fake_drive started')

    def cmd_callback(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # Integrate base pose
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # Differential drive wheel angular velocities
        v_left = self.v - 0.5 * self.w * self.wheel_separation
        v_right = self.v + 0.5 * self.w * self.wheel_separation

        left_w = v_left / self.wheel_radius
        right_w = v_right / self.wheel_radius

        self.left_pos += left_w * dt
        self.right_pos += right_w * dt

        self.publish_joint_states(now, left_w, right_w)
        self.publish_odom(now)
        self.publish_tf(now)

    def publish_joint_states(self, now, left_vel, right_vel):
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            self.left_front_joint,
            self.left_rear_joint,
            self.right_front_joint,
            self.right_rear_joint,
        ]
        msg.position = [
            self.left_pos,
            self.left_pos,
            self.right_pos,
            self.right_pos,
        ]
        msg.velocity = [
            left_vel,
            left_vel,
            right_vel,
            right_vel,
        ]
        self.joint_pub.publish(msg)

    def publish_odom(self, now):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quaternion(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w

        self.odom_pub.publish(msg)

    def publish_tf(self, now):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quaternion(self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()