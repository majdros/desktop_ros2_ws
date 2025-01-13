from setuptools import find_packages, setup

package_name = 'transform_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odom_laser_broadcaster.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majd',
    maintainer_email='majd.fa.shahrour@outlook.com',
    description='Package for broadcasting transforms between odom and laser frames',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_laser_broadcaster = transform_broadcaster.odom_laser_broadcaster:main',
        ],
    },
)
