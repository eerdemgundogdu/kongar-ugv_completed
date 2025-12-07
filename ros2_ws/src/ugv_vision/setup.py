from setuptools import setup

package_name = 'ugv_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team KONGAR',
    maintainer_email='kongar.ugv@gmail.com',
    description='Vision processing for UGV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = ugv_vision.lane_detector:main',
            'object_detector = ugv_vision.object_detector:main',
            'slam_integration = ugv_vision.slam_integration:main',
            'visual_odometry = ugv_vision.visual_odometry:main',
            'camera_driver = ugv_vision.camera_driver:main',
            'rplidar_processor = ugv_vision.rplidar_processor:main',
            'lidar_slam_bridge = ugv_vision.lidar_slam_bridge:main',
        ],
    },
)
