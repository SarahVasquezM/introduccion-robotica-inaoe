from glob import glob
import os
from setuptools import find_packages, setup

package_name = "autominy_pure_pursuit"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sarah",
    maintainer_email="sarah@example.com",
    description=(
        "Physical AutoMiny Pure Pursuit with final pose goals."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pure_pursuit = autominy_pure_pursuit.controller:main",
        ],
    },
)
