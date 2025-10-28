from setuptools import find_packages, setup
from glob import glob

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/local_planning_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoo',
    maintainer_email='smzzang21@konkuk.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'make_pathnplan = path_planner.make_pathnplan:main',
        'path_follower = path_planner.path_follower:main',
        'nav2_pathnplan = path_planner.nav2_pathnplan:main',
        'nav2_nav_to_pose = path_planner.nav2_navigate_to_pose:main',
        'local_avoid_planner = path_planner.local_avoid_planner:main',
    ],
    },
)
