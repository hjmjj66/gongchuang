from glob import glob
from setuptools import find_packages, setup

setup(
    name='agv_bringup', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agv_bringup']),
        ('share/agv_bringup', ['package.xml', 'README.md']),
        ('share/agv_bringup/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='AGV Team', maintainer_email='team@example.com', license='Apache-2.0',
    description='Supervised competition startup',
)
