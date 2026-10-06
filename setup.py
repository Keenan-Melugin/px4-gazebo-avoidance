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
        (os.path.join('share', package_name, 'config'),
         glob('config/*.rviz') + glob('config/*.yaml')
         + glob('config/*.xml')),
        (os.path.join('share', package_name, 'patches'),
         glob('patches/*.patch')),
        (os.path.join('share', package_name, 'scripts'),
         ['scripts/px4_params.sh', 'scripts/link_assets.sh']),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'x500_depth_lidar'),
         glob('models/x500_depth_lidar/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keenan Melugin',
    maintainer_email='Keenan-Melugin@users.noreply.github.com',
    description=(
        'Depth-camera obstacle avoidance for PX4 SITL in Gazebo Harmonic, '
        'flown from RViz.'
    ),
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'obstacle_distance = avoidance_sim.obstacle_distance:main',
            'rviz_bridge = avoidance_sim.rviz_bridge:main',
        ],
    },
)
