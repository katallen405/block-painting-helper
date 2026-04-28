from setuptools import find_packages, setup

package_name = 'bph_statemachine'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kat Allen',
    maintainer_email='kat.allen@tufts.edu',
    description='SMACH state machine for block painting helper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_sm_node = bph_statemachine.simplified_sm:main',
        'arm_sm_node = bph_statemachine.arm_sm:main',
        'base_sm_node = bph_statemachine.mobile_base_sm:main',
        ],
    },
)
