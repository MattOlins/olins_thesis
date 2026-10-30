# from setuptools import find_packages, setup

# package_name = 'vfh_navigation'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='kalgaonp',
#     maintainer_email='olinsmatt@github.com',
#     description='TODO: Package description',
#     license='Apache-2.0',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'vfh_navigation'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob('config/*.yaml')),
    ],

    install_requires=['setuptools', 'numpy'],
    zip_safe=True,

    maintainer='kalgaonp',
    maintainer_email='olinsmatt@gmail.com',
    description='VFH obstacle avoidance for four-wheel robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vfh_node = vfh_navigation.vfh_node:main',
        ],
    },
)