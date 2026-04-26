import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'traffic_light_nav'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Lin Tran',
    maintainer_email='lintran@nvidia.com',
    description='Autonomous Traffic Light Obedience for ROSbot 3 PRO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_detector = traffic_light_nav.signal_detector_node:main',
            'navigation_controller = traffic_light_nav.navigation_controller_node:main',
            'traffic_logger = traffic_light_nav.data_logger_node:main',
        ],
    },
)
