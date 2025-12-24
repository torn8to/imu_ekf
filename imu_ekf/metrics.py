import numpy as np 
from imu_ekf.data import Position
from typing import List

class Position:
    def __init__(self, timestamp: float = -1, position: np.ndarray = np.zeros(3)):
        self.timestamp = timestamp
        self.position = position

    def normDistance(self, other:Position) -> float:
        if isinstance(other, Position):
            return np.abs(self.position - other.position)
        else:
            throw TypeError("other must be of type Position to get norm distance")

    def __sub__(self, other: Position | float) -> float:
        if isinstance(other, Position):
            return self.position - other.position
        elif isinstance(other, float) and self.timestamp > 0:
            return self.timestamp - other
        elif isinstance(other, float):
            throw RuntimeError("other must be of type Position or float while timstamp is considered a timestamp")
        else:
            throw TypeError("other must be of type Position or float to get norm distance")
        return s 
PositionData = List[Position]
    
def nearest_neighbor_position_index(position_data: list[Position], gt_data: list[np.ndarray]) -> list[int]: 
    list_time_diff:list[float] = []
    def nearest_index(position_data:Position, gt_data:list[Position]) -> int:
        return np.argmin(np.abs(position_data.timestamp - gt_data.timestamp))
    for i in range(len(position_data)):
        list_time_diff.append(nearest_index(position_data[i], gt_data))
    return list_time_diff.index(min(list_time_diff))

def rmse_position(imu_data: list[Position], gt_data: list[Position], imu_time_offset = 0.0) -> float:
    index_list = []
    imu_data_list = []
    gt_data_list = []

    nearest_neighbor_position_index(imu_data, gt_data, imu_time_offset)
    imu_data_list.append(imu_data.x, imu_data.y, imu_data.z)
    gt_data_list.append( gt_data.x, gt_data.y, gt_data.z)
    imu_data_list = np.array(imu_data_list)
    gt_data_list = np.array(gt_data_list)
    return np.sqrt(np.mean((imu_data_list - gt_data_list)**2))

def nearest_neighbor_time_index(imu_data: list[Position], gt_data: list[Position], imu_time_offset = 0.0) -> int:
    list_time_diff:list[float] = []
    for i in range(len(imu_data)):
        list_time_diff.append(imu_data[i].header.stamp.to_sec() - imu_time_offset - gt_data[i].header.stamp.to_sec())
    return list_time_diff.index(min(list_time_diff))


def rmse_time_synchronized(imu_data: list, gt_data: list, imu_time_offset = 0.0) -> float:
    index_list = []
    imu_data_list = []
    gt_data_list = []

    for i in range(len(gt_data)):
        index_list.append(nearest_time_index(imu_data, gt_data[i], imu_time_offset))
    imu_data_list.append(imu_data.x, imu_data.y, imu_data.z)
    gt_data_list.append( gt_data.x, gt_data.y, gt_data.z)
    imu_data_list = np.array(imu_data_list)
    gt_data_list = np.array(gt_data_list)
    return np.sqrt(np.mean((imu_data_list - gt_data_list)**2))