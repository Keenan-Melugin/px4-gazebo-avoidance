from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'avoidance_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'patches'),
         glob('patches/*.patch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keenan Melugin',
    maintainer_email='Keenan-Melugin@users.noreply.github.com',
    description=(
        'Depth-camera obstacle avoidance for PX4 SITL in Gazebo Harmonic, '
        'flown from RViz.'
    ),
    license='TODO: choose a license before publishing',
    entry_points={
        'console_scripts': [
            'obstacle_distance = avoidance_sim.obstacle_distance:main',
            'rviz_bridge = avoidance_sim.rviz_bridge:main',
        ],
    },
)
