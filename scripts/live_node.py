import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from imu_ekf.imu_bias_ekf import IMUBiasEKF
from imu_ekf.data import IMUData, odometry_data, Position



import numpy as np
from scipy.spatial.transform import Rotation as R

class LiveNode(Node):
    def __init__(self):
        super().__init__('live_node')
        self.ekf = IMUBiasEKF()
        self.last_imu_timestamp = None
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odometry_data',
            self.odom_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            10)
        
        # Publisher for fused odometry
        self.filtered_odom_pub = self.create_publisher(Odometry, 'filtered_odom', 10)
        
        # Publisher for biases (using Imu message as a container for 3D vectors w/o covariance)
        self.bias_pub = self.create_publisher(Imu, 'imu_biases', 10)

    def odom_callback(self, msg: Odometry):
        # Convert to odometry_data dataclass
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos_obj = Position(
            timestamp=current_time,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            z=msg.pose.pose.position.z
        )
        
        q = msg.pose.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        
        odom = odometry_data(
            timestamp=current_time,
            position=pos_obj,
            orientation=quat
        )
        
        # Use dataclass orientation
        ori = R.from_quat(odom.orientation).as_rotvec()
        z = np.concatenate([np.array([odom.position.x, odom.position.y, odom.position.z]), ori])
        
        self.ekf.update_step(z)
        self.publish_filtered_odom(msg.header.stamp)

    def imu_callback(self, msg: Imu):
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Convert to IMUData dataclass
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        q = msg.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        
        imu = IMUData(
            timestamp=current_timestamp,
            angular_velocity=gyro,
            linear_acceleration=accel,
            orientation_quat=quat
        )

        if self.last_imu_timestamp is None:
            self.last_imu_timestamp = imu.timestamp
            return
        
        dt = imu.timestamp - self.last_imu_timestamp
        self.last_imu_timestamp = imu.timestamp
        
        if dt <= 0:
            return

        self.ekf.prediction_step(dt, imu.angular_velocity, imu.linear_acceleration, imu.orientation_quat)

    def publish_filtered_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        
        # Set position
        msg.pose.pose.position.x = self.ekf.ekf.x[0]
        msg.pose.pose.position.y = self.ekf.ekf.x[1]
        msg.pose.pose.position.z = self.ekf.ekf.x[2]
        
        # Set orientation
        rotvec = self.ekf.ekf.x[6:9]
        quat = R.from_rotvec(rotvec).as_quat()
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        # We could also set the twist (velocity) here if needed
        msg.twist.twist.linear.x = self.ekf.ekf.x[3]
        msg.twist.twist.linear.y = self.ekf.ekf.x[4]
        
        self.filtered_odom_pub.publish(msg)

        # Publish biases
        bias_msg = Imu()
        bias_msg.header.stamp = stamp
        bias_msg.header.frame_id = 'world'
        
        # We reuse the angular_velocity and linear_acceleration fields to store biases
        bias_msg.angular_velocity.x = self.ekf.ekf.x[9]
        bias_msg.angular_velocity.y = self.ekf.ekf.x[10]
        bias_msg.angular_velocity.z = self.ekf.ekf.x[11]
        
        bias_msg.linear_acceleration.x = self.ekf.ekf.x[12]
        bias_msg.linear_acceleration.y = self.ekf.ekf.x[13]
        bias_msg.linear_acceleration.z = self.ekf.ekf.x[14]
        
        self.bias_pub.publish(bias_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        