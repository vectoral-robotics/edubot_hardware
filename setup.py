import os
from glob import glob

from setuptools import find_packages, setup

package_name = "edubot_hardware"

setup(
    name=package_name,
    version="0.1.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        # ROS package index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Package manifest
        ("share/" + package_name, ["package.xml"]),
        # Optional: install any launch files for direct use
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=[
        "setuptools",
        "pyserial",  # rosdep key: python3-serial
    ],
    zip_safe=True,
    maintainer="Vectoral",
    maintainer_email="info@vectoral.ch",
    description="ROS 2 hardware and simulation backend for the EduBot robot.",
    license="PolyForm-Perimeter-1.0.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardware_node = edubot_hardware.hardware_node:main",
            "led_node = edubot_hardware.led_node:main",
        ],
    },
)
