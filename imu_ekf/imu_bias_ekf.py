import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from scipy.spatial.transform import Rotation as R
import yaml

from dataclasses import dataclass

@dataclass
class InitialCovariance:
    std_covariance: float = 1.0
    angular_velocity_covariance: float = 0.1
    linear_acceleration_covariance: float = 0.1

@dataclass
class ProcessNoise:
    process_noise_scalar: float = 0.1
    position_noise: float = 1.0
    velocity_noise: float = 0.5
    orientation_noise: float = 0.1
    gyro_bias_noise: float = 0.0001
    acc_bias_noise: float = 0.0001

@dataclass
class MeasurementNoise:
    std_covariance: float = 1.0
    angular_velocity_covariance: float = 0.1



class IMUBiasEKF:
    def __init__(self, 
        initial_covariance: InitialCovariance = InitialCovariance(),
        process_noise: ProcessNoise = ProcessNoise(),
        measurement_noise: MeasurementNoise = MeasurementNoise(),
        ):
        self.initial_covariance:InitialCovariance = initial_covariance
        self.process_noise:ProcessNoise = process_noise
        self.measurement_noise:MeasurementNoise = measurement_noise
        self.bias_converged:bool = False
        self.bias_history:list[np.ndarray] = []
        self.max_bias_iterations = 100
        self.bias_convergence_threshold = 0.001

        self.ekf:ExtendedKalmanFilter = ExtendedKalmanFilter(dim_x=15, dim_z=6)
        self.ekf.x = np.zeros(15)


        #initialize process noise covariance
        self.ekf.P = np.eye(15) * self.initial_covariance.std_covariance
        self.ekf.P[9:12,9:12] = self.ekf.P[9:12,9:12] * self.initial_covariance.angular_velocity_covariance
        self.ekf.P[12:15,12:15] = self.ekf.P[12:15,12:15] * self.initial_covariance.linear_acceleration_covariance

        #initialize process noise covariance
        self.ekf.Q = np.eye(15) * self.process_noise.process_noise_scalar
        self.ekf.Q[0:3, 0:3] *= self.process_noise.position_noise  
        self.ekf.Q[3:6, 3:6] *= self.process_noise.velocity_noise  
        self.ekf.Q[6:9, 6:9] *= self.process_noise.orientation_noise  
        self.ekf.Q[9:12, 9:12] *= self.process_noise.gyro_bias_noise  
        self.ekf.Q[12:15, 12:15] *= self.process_noise.acc_bias_noise   

        #initialize measurement noise covariance
        self.ekf.R = np.eye(6)
        self.ekf.R[0:3, 0:3] *= self.measurement_noise.std_covariance
        self.ekf.R[3:6, 3:6] *= self.measurement_noise.angular_velocity_covariance

    def check_bias_convergence(self):
        current_bias:np.ndarray = np.concatenate([self.ekf.x[9:12], self.ekf.x[12:15]])
        self.bias_history.append(current_bias)
        if len(self.bias_history) > self.max_bias_iterations:
            array_bias: np.ndarray = np.array(self.bias_history[-self.max_bias_iterations:])
            array_bias_std: np.ndarray = np.std(array_bias, axis=0)
            if np.all(array_bias_std < self.bias_convergence_threshold):
                self.bias_converged = True

    def load_bias_from_file(self, file_path="imu_filter_bias.yaml"):
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            self.ekf.x[9:12] = yaml_data['gyro_bias']
            self.ekf.x[12:15] = yaml_data['acc_bias']

    def save_bias_to_file(self, file_path="imu_filter_bias.yaml"):
        """
        Save the current bias to a YAML file.
        """
        with open(file_path, 'w') as file:
            yaml.dump({'gyro_bias': self.ekf.x[9:12], 'acc_bias': self.ekf.x[12:15]}, file)

    def update_step(self, z):
        """
        EKF Update step using external odometry data.
        z: [x, y, z, roll, pitch, yaw] or [x, y, z, rx, ry, rz]
        Currently assumes z matches the position and orientation parts of the state.
        """
        # Measurement matrix H
        # z = [pos, ori]
        # x = [pos, vel, ori, gyro_bias, acc_bias]
        H = np.zeros((6, 15))
        H[0:3, 0:3] = np.eye(3)  # Position
        H[3:6, 6:9] = np.eye(3)  # Orientation

        def hx(x):
            # Measurement model: return position and orientation from state
            return np.concatenate([x[0:3], x[6:9]])

        self.ekf.update(z, H=H, Hx=hx)
        self.check_bias_convergence()

    def prediction_step(self, dt, angular_velocity, linear_acceleration, orientation_quat=None):
        """
        EKF Prediction step using IMU data.
        angular_velocity: [wx, wy, wz]
        linear_acceleration: [ax, ay, az]
        dt: time step
        orientation_quat: [x, y, z, w] (Known orientation from IMU)
        """
        # 1. State Propagation (Prediction)
        pos = self.ekf.x[0:3]
        vel = self.ekf.x[3:6]
        ori_vec = self.ekf.x[6:9]
        gyro_bias = self.ekf.x[9:12]
        acc_bias = self.ekf.x[12:15]

        # Correct IMU measurements
        w_corr = angular_velocity - gyro_bias
        a_corr = linear_acceleration - acc_bias

        # Current Rotation Matrix
        rot = R.from_rotvec(ori_vec)
        rot_mat = rot.as_matrix()

        # Update Orientation using gyro integration
        delta_rot = R.from_rotvec(w_corr * dt)
        new_rot = rot * delta_rot
        new_ori_vec = new_rot.as_rotvec()

        # Update Position and Velocity
        gravity = np.array([0, 0, -9.81])
        acc_world = rot_mat @ a_corr + gravity

        new_pos = pos + vel * dt + 0.5 * acc_world * dt**2
        new_vel = vel + acc_world * dt

        # Apply Prediction to state
        self.ekf.x[0:3] = new_pos
        self.ekf.x[3:6] = new_vel
        self.ekf.x[6:9] = new_ori_vec

        # Compute Jacobian F
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        
        a_skew = np.array([
            [0, -a_corr[2], a_corr[1]],
            [a_corr[2], 0, -a_corr[0]],
            [-a_corr[1], a_corr[0], 0]
        ])
        F[3:6, 6:9] = -rot_mat @ a_skew * dt
        F[3:6, 12:15] = -rot_mat * dt

        F[6:9, 9:12] = -np.eye(3) * dt # Orientation / Gyro Bias
        w_skew = np.array([
            [0, -w_corr[2], w_corr[1]],
            [w_corr[2], 0, -w_corr[0]],
            [-w_corr[1], w_corr[0], 0]
        ])
        F[6:9, 6:9] = np.eye(3) - w_skew * dt

        self.ekf.predict(F=F, Q=self.ekf.Q)

        # 2. Known Orientation Update (Internal)
        # If a known orientation is provided, use it to correct the state and gyro bias
        if orientation_quat is not None:
            # We treat the orientation as a measurement z = [rx, ry, rz]
            z_ori = R.from_quat(orientation_quat).as_rotvec()
            
            # Measurement matrix for orientation part of the state
            H_ori = np.zeros((3, 15))
            H_ori[:, 6:9] = np.eye(3)
            
            def hx_ori(x):
                return x[6:9]
            
            # Use a smaller measurement noise for the known orientation if desired
            # R_ori = self.ekf.R[3:6, 3:6]
            self.ekf.update(z_ori, H=H_ori, Hx=hx_ori)
