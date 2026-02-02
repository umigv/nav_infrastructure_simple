from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*')),
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan Liao',
    maintainer_email='ryanliao@umich.edu',
    description='Localization helper nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "gps_origin_initializer = localization.gps_origin_initializer:main",
        ],
    },
)
