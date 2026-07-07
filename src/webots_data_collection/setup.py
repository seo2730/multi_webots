import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'webots_data_collection'

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
    maintainer='root',
    maintainer_email='seo2730@naver.com',
    description='Camera-LiDAR synchronized data collection and KITTI dataset conversion',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_lidar_data_collector = webots_data_collection.cam_lidar_data_collector:main',
        ],
    },
)
