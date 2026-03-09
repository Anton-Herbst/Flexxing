from setuptools import find_packages, setup
from glob import glob

package_name = 'sensor'

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
    maintainer_email='anton@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'calibrate_IMU_tip = sensor.calibrate_IMU_tip:main',
            'calibrate_IMU_all = sensor.calibrate_IMU_all:main',
            'calibrate_mag_tip = sensor.calibrate_mag_tip:main',
            'calibrate_mag_all = sensor.calibrate_mag_all:main',
            'publish_IMU_acc_tip = sensor.publish_IMU_acc_tip:main',
            'publish_IMU_acc_all = sensor.publish_IMU_acc_all:main',
            'publish_IMU_grav_tip = sensor.publish_IMU_grav_tip:main',
            'publish_IMU_grav_all = sensor.publish_IMU_grav_all:main',
        ],
    },
)
