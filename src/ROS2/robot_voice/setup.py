from setuptools import setup
import os
from glob import glob

package_name = 'robot_voice'

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
    maintainer_email='user@todo.todo',
    description='Voice Broadcast &amp; Voice Command Node (SU-03T)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_node = robot_voice.voice_node:main'
        ],
    },
)
