#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class DataPointRecorder(Node):
    def __init__(self):
        super().__init__('data_point_recorder')

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.imu_data = []

        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )
        self.odometry_data = []

    def imu_callback(self, msg: Imu):
        self.imu_data.append(msg)

    def odometry_callback(self, msg: Odometry):
        self.odometry_data.append(msg)

    def save_data(self):
        with open('imu_data.csv', 'w') as imu_file:
            for data in self.imu_data:
                imu_file.write(f"{data.header.stamp.sec},{data.header.stamp.nanosec},
                {data.linear_acceleration.x},{data.linear_acceleration.y},{data.linear_acceleration.z},
                {data.angular_velocity.x},{data.angular_velocity.y},{data.angular_velocity.z}\n")

        with open('odometry_data.csv', 'w') as odometry_file:
            for data in self.odometry_data:
                odometry_file.write(f"{data.header.stamp.sec},{data.header.stamp.nanosec},
                        {data.pose.pose.position.x},{data.pose.pose.position.y},{data.pose.pose.position.z},
                        {data.pose.pose.orientation.x},{data.pose.pose.orientation.y},{data.pose.pose.orientation.z},{data.pose.pose.orientation.w}\n")

    def destroy_node(self):
        self.save_data()
        self.imu_subscription.destroy()
        self.odometry_subscription.destroy()
        super().destroy_node()


def run():
    rclpy.init()
    node = DataPointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run()