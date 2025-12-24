import numpy as np

from dataclasses import dataclass

@dataclass
class Position:
    timestamp: float
    x: float
    y: float
    z: float


@dataclass
class IMUData:
    timestamp: float
    angular_velocity: np.ndarray  # [wx, wy, wz]
    linear_acceleration: np.ndarray  # [ax, ay, az]
    orientation_quat: np.ndarray # [x, y, z, w]


@dataclass
class odometry_data:
    timestamp: float
    position: Position 
    orientation: np.ndarray


def load_imu_data(file_path: str) -> list[Position]:
    with open(file_path, 'rb') as file:
        data = np.load(file)
        return [Position(timestamp=data['timestamp'][i], x=data['position'][i, 0], y=data['position'][i, 1], z=data['position'][i, 2]) for i in range(len(data['timestamp']))]


def load_odometry_data(file_path: str) -> list[odometry_data]:
    with open(file_path, 'rb') as file:
        data = np.load(file)
        return [odometry_data(timestamp=data['timestamp'][i], position=Position(timestamp=data['timestamp'][i], x=data['position'][i, 0], y=data['position'][i, 1], z=data['position'][i, 2]), orientation=data['orientation'][i]) for i in range(len(data['timestamp']))]


    