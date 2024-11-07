from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'navigation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # アクションファイルのインストール
        (os.path.join('share', package_name), glob('action/*.action')),
        # launch
        ('share/' + package_name + '/launch', ['launch/navigation_control.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'stop_flag = navigation_control.stop_flag:main',
        "button = navigation_control.button:main",
        "client = navigation_control.client:main",
        "client2 = navigation_control.client2:main",
        "client3 = navigation_control.client3:main",
        "server = navigation_control.server:main",
        "waypoint_monitor = navigation_control.waypoint_monitor:main",
        "trafficlight_waypoint_monitor = navigation_control.trafficlight_waypoint_monitor:main",
        "judge_trafficlight = navigation_control.judge_trafficlight:main",
        'camera_publisher = navigation_control.camera_publisher:main',
        ],
    },
)
