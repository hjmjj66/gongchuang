from glob import glob
from setuptools import find_packages, setup

setup(
    name='agv_mission', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agv_mission']),
        ('share/agv_mission', ['package.xml', 'README.md']),
        ('share/agv_mission/config', glob('config/*.yaml')),
        ('share/agv_mission/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='AGV Team', maintainer_email='team@example.com', license='Apache-2.0',
    description='Competition state machine and base velocity forwarding',
    entry_points={'console_scripts': [
        'mission_node = agv_mission.mission_node:main',
    ]},
)
