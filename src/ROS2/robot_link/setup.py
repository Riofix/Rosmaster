from setuptools import setup
import os
from glob import glob

package_name = 'robot_link'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='1789518998@qq.com',
    description='Physical IO link layer for Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hotspot_node = robot_link.hotspot_node:main',
            'tcp_server_node = robot_link.tcp_server_node:main',
            'serial_node = robot_link.serial_node:main',
        ],
    },
)
