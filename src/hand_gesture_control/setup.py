from setuptools import find_packages, setup

package_name = 'hand_gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [
            'scripts/camera_node.py',
            'scripts/hand_tracking_node.py',
            'scripts/gesture_control_node.py',
            'scripts/twist_stamper.py',
            ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majd',
    maintainer_email='majd.fa.shahrour@outlook.com',
    description='control the robot through hand gesture with OpenCV and MediaPipe',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = scripts.camera_node:main',
            'hand_tracking_node = scripts.hand_tracking_node:main',
            'gesture_control_node = scripts.gesture_control_node:main',
            'twist_stamper_node = scripts.twist_stamper:main',
        ],
    },
)