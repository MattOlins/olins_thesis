## Remade Formatter
import os
from glob import glob
from setuptools import setup

package_name = "my_robot_bringup"
setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kalgaonp",
    maintainer_email="user@example.com",
    description="Four-wheel robot simulation and bringup",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
import os

from glob import glob
from setuptools import setup


package_name = 'my_robot_bringup'


setup(
    name=package_name,
    version='0.0.1',

    packages=[package_name],

    data_files=[

        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'urdf'
            ),
            glob('urdf/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='kalgaonp',
    maintainer_email='your_email@example.com',

    description='Bringup for VFH thesis robot',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [],
    },
)

## Original Formatter
# from setuptools import find_packages, setup

# package_name = 'my_robot_bringup'

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
