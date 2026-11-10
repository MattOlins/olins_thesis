"""Package simulation, localization, navigation launches, and scenario fixtures."""

from glob import glob
import os

from setuptools import setup

package_name = 'my_robot_bringup'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config', 'scenarios'),
         glob('config/scenarios/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='molins',
    maintainer_email='olinsmatt@gmail.com',
    description='Four-wheel skid-steer robot bringup and navigation scenarios',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
