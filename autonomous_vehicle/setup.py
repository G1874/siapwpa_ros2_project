from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_vehicle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='gniedziolka@student.agh.edu.pl',
    description='Package description',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'warp_perspective = autonomous_vehicle.warp_perspective:main',
            'helper_node = autonomous_vehicle.helper_node:main',
            'image_binarizer = autonomous_vehicle.image_binarizer:main',
            'binarization_slider = autonomous_vehicle.binarization_slider:main',
            'image_skeletonizer = autonomous_vehicle.image_skeletonizer:main',
            'motion_control = autonomous_vehicle.motion_control:main',
            'Recognizing = autonomous_vehicle.Recognizing:main',
            'color_slider = autonomous_vehicle.color_slider:main',
            'road_detector = autonomous_vehicle.road_detector:main',
            'pedestrians = autonomous_vehicle.pedestrians:main',
        ],
    },
)
