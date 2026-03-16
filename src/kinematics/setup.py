from setuptools import find_packages, setup

package_name = 'kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='anton-herbst@gmx.net',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'G2X_imu_acc_all = kinematics.G2X_imu_acc_all:main',
            'G2X_imu_acc_tip = kinematics.G2X_imu_acc_tip:main',
            'G2L_imu_acc_all = kinematics.G2L_imu_acc_all:main',
            'G2L_trajectory = kinematics.G2L_trajectory:main',
        ],
    },
)
