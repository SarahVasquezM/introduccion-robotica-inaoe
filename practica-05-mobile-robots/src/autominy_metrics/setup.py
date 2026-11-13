from glob import glob
import os
from setuptools import find_packages, setup

package_name = "autominy_metrics"

setup(
    name=package_name,
    version="0.2.0",
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
        "Passive AutoMiny metrics for position and orientation goals."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "metrics_logger = autominy_metrics.metrics_logger:main",
        ],
    },
)
