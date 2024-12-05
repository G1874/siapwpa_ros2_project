from setuptools import setup
import os
from glob import glob
package_name = 'what'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartosz Starzyk',
    maintainer_email='bartosz.starzyk@gmail.com',
    description='Some garbage code',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'image_binarizer = what.image_binarizer:main',
            'drive_node = what.drive_node:main',
            'birdseye_transform = what.birdseye_transform:main',
            'birdseye_slider = what.birdseye_slider:main',
            'binarization_slider = what.binarization_slider:main'
        ],
    },
)
