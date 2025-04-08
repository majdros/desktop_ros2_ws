from setuptools import find_packages, setup

package_name = 'my_robot_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bno055.launch.py',
            'launch/rplidar.launch.py',
            'launch/camera.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/rplidar_params.yaml',
            'config/bno055_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majd',
    maintainer_email='majd.fa.shahrour@outlook.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = my_robot_sensors.camera_node:main',
            'bno055 = my_robot_sensors.bno055_node:main'
        ],
    },
)