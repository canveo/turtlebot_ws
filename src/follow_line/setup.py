import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'follow_line'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    # install_requires=['setuptools'],
    zip_safe=True,
    maintainer='canveo',
    maintainer_email='canveo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_line_node = follow_line.follow_line_node:main',
            'pilotnet_turtle = follow_line.pilotnet_turtle:main',
        ],
    },
)
