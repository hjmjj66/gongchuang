from glob import glob

from setuptools import find_packages, setup

setup(
    name='agv_perception',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agv_perception']),
        ('share/agv_perception', ['package.xml']),
        ('share/agv_perception/config', glob('config/*.yaml')),
        ('share/agv_perception/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='team@example.com',
    description='USB camera QR scanner using preprocessing and ZBar',
    license='Apache-2.0',
    entry_points={'console_scripts': ['qr_node = agv_perception.qr_node:main']},
)
