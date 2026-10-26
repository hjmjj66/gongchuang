from glob import glob

from setuptools import find_packages, setup


setup(
    name='agv_cboard',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agv_cboard']),
        ('share/agv_cboard', ['package.xml', 'README.md']),
        ('share/agv_cboard/config', glob('config/*.yaml')),
        ('share/agv_cboard/launch', glob('launch/*.launch.py')),
        ('share/agv_cboard/udev', glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='team@example.com',
    description='USB CDC bridge for the RoboMaster C-board chassis protocol.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cboard_bridge_node = agv_cboard.cboard_bridge_node:main',
            'cboard_diagnostics = agv_cboard.diagnostics:main',
        ],
    },
)
