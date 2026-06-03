from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cognitive_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mikesmalbroek@hotmail.com',
    description='Cognitive robot package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Main service node — runs on the laptop, reads the clock via OCR.
            'read_time_service = cognitive_robot.read_time_service:main',

            # Debug tool — manually save camera frames to test the OCR offline.
            'make_photo_for_testing_algorithm = cognitive_robot.make_photo_for_testing_algorithm:main',

            # One-shot ArUco stationsdetectie — pakt één frame en detecteert markers.
            'detect_station_service = cognitive_robot.detect_station_service:main',
        ],
    },
)
