from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_runtime'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.json'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiexiang',
    maintainer_email='12313401@mail.sustech.edu.cn',
    description='Robot runtime device and state layer.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'runtime_node = robot_runtime.runtime_node:main',
            'step_climb_forward = robot_runtime.step_climb_forward:main',
        ],
    },
)
