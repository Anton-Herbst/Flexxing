from setuptools import find_packages, setup
from glob import glob

package_name = 'visualize'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # reference the launch files
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='anton@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'visualize_IMU_acc_tip = visualize.visualize_IMU_acc_tip:main',
        ],
    },
)
