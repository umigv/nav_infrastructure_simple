from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "nav_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(Path("share") / package_name / "launch"), glob("launch/*")),
        (str(Path("share") / package_name / "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ethan Hardy",
    maintainer_email="hardyem@umich.edu",
    description="Launch files and shared configuration for the navigation stack",
    license="Apache-2.0",
    extras_require={},
    entry_points={
        "console_scripts": [],
    },
)
