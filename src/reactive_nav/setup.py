import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'reactive_nav'

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
    description='Reactive autonomous navigation with obstacle avoidance for ROSbot 3 PRO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion = reactive_nav.sensor_fusion_node:main',
            'reactive_navigator = reactive_nav.reactive_navigator_node:main',
            'safety_monitor = reactive_nav.safety_monitor_node:main',
            'data_logger = reactive_nav.data_logger_node:main',
        ],
    },
)
