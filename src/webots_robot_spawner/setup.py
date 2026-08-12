import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'webots_robot_spawner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 편대 매니페스트. 여기 파일을 추가하면 fleet:= 인자로 바로 고를 수 있다.
        (os.path.join('share', package_name, 'config', 'fleet'),
            glob('config/fleet/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='seo2730@naver.com',
    description='실행 중인 Webots 시뮬레이션에 로봇을 원하는 위치 또는 맵의 빈 공간에 소환',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spawn_supervisor = webots_robot_spawner.spawn_supervisor:main',
        ],
    },
)
