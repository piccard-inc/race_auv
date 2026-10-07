from glob import glob

from setuptools import find_packages, setup

package_name = 'race_auv_docking_planner'

setup(
    name=package_name,
    version='1.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piccard Inc.',
    maintainer_email='bora@xbora.tv',
    description='RACE AUV docking planner, protocol v1.5: a staged approach on the AprilTag-fused station dock point',
    license='Copyright Piccard Inc.; licence pending founder decision',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_planner = race_auv_docking_planner.node:main',
        ],
    },
)
