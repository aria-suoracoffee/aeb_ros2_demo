import os
from glob import glob

from setuptools import find_packages, setup

package_name = "aeb_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aria",
    maintainer_email="ariars2@illinois.edu",
    description="Automatic Emergency Braking demo in ROS 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = aeb_demo.sim_node:main",
            "perception_node = aeb_demo.perception_node:main",
            "aeb_node = aeb_demo.aeb_node:main",
            "driver_node = aeb_demo.driver_node:main",
        ],
    },
)
