from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot4_python_tutorials'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join("share", package_name), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rkreinin',
    maintainer_email='rkreinin@clearpathrobotics.com',
    description='TurtleBot 4 Python Tutorials',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_first_python_node = \
                turtlebot4_python_tutorials.turtlebot4_first_python_node:main',
            'nav_to_pose = turtlebot4_python_tutorials.nav_to_pose:main',
            'nav_through_poses = turtlebot4_python_tutorials.nav_through_poses:main',
            'follow_waypoints = turtlebot4_python_tutorials.follow_waypoints:main',
            'create_path = turtlebot4_python_tutorials.create_path:main',
            'patrol_loop = turtlebot4_python_tutorials.patrol_loop:main',
            'mail_delivery = turtlebot4_python_tutorials.mail_delivery:main',
            'random_movement = turtlebot4_python_tutorials.random_movement:main',
            'dummy_odom_pub = turtlebot4_python_tutorials.dummy_odom_pub:main',
            'test_vicon = turtlebot4_python_tutorials.test_vicon:main',
            'tf_odom_broadcaster = turtlebot4_python_tutorials.tf_odom_broadcaster:main',
        ],
    },
)
