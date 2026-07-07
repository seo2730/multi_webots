from setuptools import find_packages, setup

package_name = 'webots_goal_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='seo2730@naver.com',
    description='External goal sources (web UI clicks, Gemini AI) relayed into Nav2 goals',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_goal_relay = webots_goal_bridge.web_goal_relay:main',
            # 아직 연동 완료 안됨
            #'gemini_goal_assigner = webots_goal_bridge.gemini_goal_assigner:main',
        ],
    },
)
