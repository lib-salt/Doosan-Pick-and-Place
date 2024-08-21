from setuptools import find_packages, setup
from glob import glob

package_name = 'sort_seg'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        # ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minsoo Song',
    maintainer_email='minsoo.song@doosan.com',
    description='sort_seg',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = sort_seg.move_robot:main',
            'test = sort_seg.test:main',
            'connect_robot = sort_seg.connect_robot:main',
            'connect_test = sort_seg.connect_test:main',
            'connection = sort_seg.connection:main',
            'color_detect = sort_seg.color_detect_node:main',
            'frame_listener_aruco = sort_seg.frame_listener_aruco:main',
            'frame_listener_robot = sort_seg.frame_listener_robot:main',
            'camera_to_aruco = sort_seg.camera_to_aruco:main',
            'aruco_to_robot = sort_seg.aruco_to_robot:main',
            'cam_map = sort_seg.cam_map:main'
        ],
    },
)
