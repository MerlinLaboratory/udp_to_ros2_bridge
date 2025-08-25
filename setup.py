from setuptools import setup, find_packages

package_name = 'udp_to_ros2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_to_ros2_bridge_launch.py']),
        ('share/' + package_name + '/config', ['config/comm_param.yaml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Marco-Faroni',
    maintainer_email='marco.faroni@polimi.it',
    description='Generic UDP to ROS2 bridge publishing UDP messages as strings',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'udp_to_ros2_bridge = udp_to_ros2_bridge.udp_to_ros2_bridge:main',
        ],
    },
)
