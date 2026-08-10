import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'webots_map_merge'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='root',
    maintainer_email='seo2730@naver.com',
    description='마스터 관제 컨테이너에서 로봇별 SLAM 맵을 world 프레임 기준 전역 맵으로 병합',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_merger = webots_map_merge.map_merger:main',
            'robot_registrar = webots_map_merge.robot_registrar:main',
        ],
    },
)
