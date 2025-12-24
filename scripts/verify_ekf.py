import numpy as np
from imu_ekf.imu_bias_ekf import IMUBiasEKF, InitialCovariance, ProcessNoise, MeasurementNoise
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def test_ekf_static():
    print("Testing EKF with static IMU data...")
    # Initialize EKF
    ekf = IMUBiasEKF()
    
    dt = 0.01
    total_time = 5.0
    steps = int(total_time / dt)
    
    # Static IMU data with some bias
    true_acc_bias = np.array([0.1, -0.05, 0.02])
    true_gyro_bias = np.array([0.01, -0.01, 0.005])
    
    # Simulated IMU readings (accel includes gravity)
    gravity = np.array([0, 0, 9.81]) # Body frame z is up
    
    pos_history = []
    bias_history = []
    
    # Assume world orientation is aligned with body for simplicity
    q_identity = np.array([0, 0, 0, 1])
    
    for i in range(steps):
        # Simulated readings
        acc = gravity + true_acc_bias + np.random.normal(0, 0.01, 3)
        gyro = true_gyro_bias + np.random.normal(0, 0.001, 3)
        
        ekf.prediction_step(dt, gyro, acc, q_identity)
        
        # Periodic update from "perfect" odometry at 0,0,0
        if i % 10 == 0:
            z = np.zeros(6) # Position [0,0,0] and Orientation [0,0,0]
            ekf.update_step(z)
            
        pos_history.append(ekf.ekf.x[0:3].copy())
        bias_history.append(ekf.ekf.x[12:15].copy())
    
    print(f"Final estimated accel bias: {ekf.ekf.x[12:15]}")
    print(f"True accel bias: {true_acc_bias}")
    
    return np.array(pos_history), np.array(bias_history), true_acc_bias

if __name__ == "__main__":
    pos, bias, true_bias = test_ekf_static()
    
    plt.figure(figsize=(12, 5))
    
    plt.subplot(1, 2, 1)
    plt.plot(pos)
    plt.title("Estimated Position (should stay near 0)")
    plt.legend(['x', 'y', 'z'])
    
    plt.subplot(1, 2, 2)
    plt.plot(bias)
    plt.axhline(y=true_bias[0], color='r', linestyle='--', alpha=0.5)
    plt.axhline(y=true_bias[1], color='g', linestyle='--', alpha=0.5)
    plt.axhline(y=true_bias[2], color='b', linestyle='--', alpha=0.5)
    plt.title("Estimated Accel Bias")
    plt.legend(['ax', 'ay', 'az'])
    
    plt.savefig('ekf_test_result.png')
    print("Test result saved to ekf_test_result.png")
