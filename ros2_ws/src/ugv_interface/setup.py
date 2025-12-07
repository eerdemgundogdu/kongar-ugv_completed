from setuptools import setup

package_name = 'ugv_interface'

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
    description='Interface package for UGV Serial Communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = ugv_interface.serial_bridge:main',
            'ros2_bridge = ugv_interface.ros2_bridge:main',
        ],
    },
)
