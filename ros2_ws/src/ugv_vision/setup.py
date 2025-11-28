from setuptools import setup

package_name = 'ugv_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vision processing for UGV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = ugv_vision.lane_detector:main',
            'object_detector = ugv_vision.object_detector:main',
            'slam_integration = ugv_vision.slam_integration:main',
            'visual_odometry = ugv_vision.visual_odometry:main',
        ],
    },
)
