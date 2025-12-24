from imu_ekf.data import odometry_data, imu_data from imu_ekf.imu_bias_ekf import IMUBiasEKF import typer


@app.command()
def main(imu_data_file: str, odometry_data_file: str, imu_bias_offset: float):
    imu_data = open(imu_data_file, 'rb')
    odometry_data = open(odometry_data_file, 'rb')