import os
from glob import glob

from setuptools import setup

package_name = 'completed_scripts_visualization'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*_launch.py"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='In this package, you will find python launch files \
        that launch specific node configurations, \
        as indicated in the name of the launch files for visualization \
        of Unitree H1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
