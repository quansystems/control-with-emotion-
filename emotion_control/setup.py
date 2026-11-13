from glob import glob
from setuptools import find_packages, setup

setup(
    name='emotion_control',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/emotion_control']),
        ('share/emotion_control', ['package.xml', 'README.md', 'requirements.txt']),
        ('share/emotion_control/config', glob('config/*.yaml')),
        ('share/emotion_control/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace Maintainer',
    maintainer_email='maintainer@example.com',
    description='Control TurtleBot4 with six facial expressions using DeepFace.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'emotion_control = emotion_control.node:main',
    ]},
)
