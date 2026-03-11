from setuptools import find_packages, setup

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
            'publish_imu_acc_tip = sensor.publish_imu_acc_tip:main',
            'publish_imu_acc_all = sensor.publish_imu_acc_all:main',
            'generalized_coords_imu_acc_tip = sensor.generalized_coords_imu_acc_tip:main',
            'generalized_coords_imu_acc_all = sensor.generalized_coords_imu_acc_all:main',
        ],
    },
)
