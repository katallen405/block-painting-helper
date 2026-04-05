from setuptools import setup, find_packages
import os
from glob import glob

package_name = "person_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # Install launch files
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        # Install config files
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="YOLOv8-pose person tracker for robot arm collision avoidance",
    license="MIT",
    entry_points={
        "console_scripts": [
            "person_tracker_node = person_tracker.person_tracker_node:main",
        ],
    },
)
