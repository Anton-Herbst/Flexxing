from setuptools import find_packages, setup
from glob import glob

package_name = 'controller'

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
            'PI_controller = controller.PI_controller:main',
            'publish_tendon_error = controller.calculate_tendon_error:main',
            'trajectory_gen = controller.trajectory_gen:main',
        ],
    },
)