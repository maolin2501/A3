from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rs_a3_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wy',
    maintainer_email='user@example.com',
    description='RS-A3 视觉抓取系统 - RealSense D435 物体检测与视觉伺服',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = rs_a3_vision.camera_node:main',
            'object_detector_node = rs_a3_vision.object_detector:main',
            'visual_servo_node = rs_a3_vision.visual_servo:main',
            'grasp_manager_node = rs_a3_vision.grasp_manager:main',
            'hand_eye_calibration_node = rs_a3_vision.hand_eye_calibration:main',
        ],
    },
)
