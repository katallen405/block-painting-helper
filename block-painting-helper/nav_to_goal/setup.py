from setuptools import setup

package_name = "nav_to_goal"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Robot-agnostic Nav2 SLAM navigation node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "navigator_node = nav_to_goal.navigator_node:main",
        ],
    },
)
