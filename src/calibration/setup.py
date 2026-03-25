from setuptools import find_packages, setup
from glob import glob

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, 
            ['package.xml']),
        # reference the config files
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
        # reference the launch files
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='anton-herbst@gmx.net',
    description='Package only dedicated to launch calibration algorithms that store transformation matrices in  ~/.ros/calibration/*.yaml files.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_tip = calibration.imu_tip:main',
            'imu_all = calibration.imu_all:main',
        ],
    },
)
