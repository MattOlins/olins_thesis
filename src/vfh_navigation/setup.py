"""Install the modular navigation package and its shared configuration."""

import os

from setuptools import find_packages, setup

package_name = 'vfh_navigation'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'VALIDATION.md']),
        # Colcon re-evaluates setup.py in the build directory, where a glob
        # can include stale/dangling symlinks from renamed configuration files.
        (os.path.join('share', package_name, 'config'), ['config/vfh_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='kalgaonp',
    maintainer_email='olinsmatt@gmail.com',
    description='Modular 120-degree VFH and occupancy-grid A* navigation',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['vfh_node = vfh_navigation.vfh_node:main']},
)
