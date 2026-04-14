from setuptools import setup
import os

package_name = 'robot_protocol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='user@todo.todo',
    description='Protocol parser and packer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_parser = robot_protocol.chassis_parser_node:main',
            'chassis_packer = robot_protocol.chassis_packer_node:main',
            'crane_parser = robot_protocol.crane_parser_node:main',
            'crane_packer = robot_protocol.crane_packer_node:main'
        ],
    },
)
