import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'spacenav_arm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='namanjain',
    maintainer_email='namanjain@todo.todo',
    description='SpaceMouse teleop for the Lynxmotion SES-Pro arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_bridge = spacenav_arm_bridge.twist_bridge:main',
            'spacemouse_teleop = spacenav_arm_bridge.spacemouse_teleop:main',
        ],
    },
)
