from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'slam_odin_bridge'

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
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiexiang',
    maintainer_email='12313401@mail.sustech.edu.cn',
    description='Bridge Odin external localization outputs into robot runtime topics.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odin_localization_bridge = slam_odin_bridge.odin_localization_bridge:main',
            'pcd_map_publisher = slam_odin_bridge.pcd_map_publisher:main',
        ],
    },
)
