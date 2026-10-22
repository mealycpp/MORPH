import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'morphing_drone'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CPP ECE',
    maintainer_email='you@cpp.edu',
    description='Morphing quadcopter simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'flight_controller = morphing_drone.flight_controller:main',
            'morph_controller = morphing_drone.morph_controller:main',
            'data_logger = morphing_drone.data_logger:main',
            'analyze_flight_data = morphing_drone.analyze_flight_data:main',
            'run_morph_sequence = morphing_drone.run_morph_sequence:main',
            'run_obstacle_course = morphing_drone.run_obstacle_course:main',
            'rl_controller = morphing_drone.ml.rl_controller:main',
            'gazebo_drone_bridge = morphing_drone.gazebo_drone_plugin:main',
            'teleop_controller = morphing_drone.teleop_controller:main',
        ],
    },
)
