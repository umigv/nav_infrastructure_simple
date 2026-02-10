from setuptools import find_packages, setup

package_name = 'self_drive_goal_selection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan Liao',
    maintainer_email='ryanliao@umich.edu',
    description='Goal selection for self drive contest',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'self_drive_goal_selection = self_drive_goal_selection.self_drive_goal_selection:main'
        ],
    },
)
