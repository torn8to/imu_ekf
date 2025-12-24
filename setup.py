from setuptools import find_packages, setup

package_name = 'imu_ekf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Rogers',
    maintainer_email='nathanroger314@gmail.com',
    description='a package for imu ekf and odometry data recording \
                to improve imu integration with unknown biases',
    license='MIT',
    entry_points={
        'console_scripts': [
            'record_data_points = imu_ekf.scripts.record_data_points:main',
        ],
    },
)
