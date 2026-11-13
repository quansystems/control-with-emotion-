from glob import glob

from setuptools import find_packages, setup


setup(
    name='robot_node',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_node']),
        ('share/robot_node', ['package.xml']),
        ('share/robot_node/licenses', ['licenses/LICENSE']),
        ('share/robot_node/config', glob('config/*.yaml')),
        ('share/robot_node/launch', glob('launch/*.launch.py')),
        ('share/robot_node/semantic_map', glob('../Map Partitioning/generated/*')),
        ('share/robot_node/rviz', glob('../Voice-Controlled/*.rviz')),
        ('share/robot_node', ['requirements-voice.txt']),
        ('share/robot_node', ['requirements-yolo.txt']),
        ('share/robot_node', ['requirements-gesture.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace Maintainer',
    maintainer_email='maintainer@example.com',
    description='Keyboard teleoperation for TurtleBot 4.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robot_node.teleop_keyboard:main',
            'waypoint_navigator = robot_node.waypoint_navigator:main',
            'object_detector = robot_node.object_detector:main',
            'gesture_recognition_node = robot_node.gesture_recognition_node:main',
            'robot_behavior_node = robot_node.robot_behavior_node:main',
            'map_partitioning = robot_node.map_partitioning:main',
            'map_region_editor = robot_node.map_region_editor:main',
            'voice_navigation = robot_node.voice_navigation:main',
            'speech_recognition = robot_node.speech_recognition:main',
        ],
    },
)
