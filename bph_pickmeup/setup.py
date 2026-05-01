from setuptools import find_packages, setup

package_name = 'bph_pickmeup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Katherine (Kat) Allen',
    maintainer_email='kat.allen@tufts.edu',
    description='Moveit wrapper for action server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bph_pickmeup_actionserver = bph_pickmeup.bph_pickmeup_action_server:main',
            'bph_pickmeup_client = bph_pickmeup.bph_pickmeup_client:main',
        ],
    },
)
