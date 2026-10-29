from glob import glob
from setuptools import find_packages, setup

setup(
    name='agv_screen', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agv_screen']),
        ('share/agv_screen', ['package.xml', 'README.md']),
        ('share/agv_screen/config', glob('config/*.yaml')),
        ('share/agv_screen/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'], zip_safe=True,
    maintainer='AGV Team', maintainer_email='team@example.com', license='Apache-2.0',
    description='Serial screen configuration and task-code display for the competition',
    entry_points={'console_scripts': ['screen_node = agv_screen.screen_node:main']},
)
