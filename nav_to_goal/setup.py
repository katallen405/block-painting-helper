from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'nav_to_goal'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+ package_name]),
        ('share/'+ package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kat Allen',
    maintainer_email='kat.allen@tufts.edu',
    description='Robot-agnostic Nav2 + SLAM navigation node for a single depth camera.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_node = nav_to_goal.navigator_node:main',
            'turtlebot_bridge = nav_to_goal.turtlebot_bridge:main',
        ],
    },
)
