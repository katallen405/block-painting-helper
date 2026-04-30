from setuptools import find_packages, setup

package_name = 'bph_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kat Allen',
    maintainer_email='kat.allen@tufts.edu',
    description='identify location of objects in the workspace',
    license='TBD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_picker = bph_perception.color_picker_node:main'
        ],
    },
)
