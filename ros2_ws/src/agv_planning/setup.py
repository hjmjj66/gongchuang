from glob import glob

from setuptools import find_packages, setup


setup(
    name='agv_planning',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agv_planning']),
        ('share/agv_planning', ['package.xml', 'README.md']),
        ('share/agv_planning/config', glob('config/*.yaml')),
        ('share/agv_planning/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='team@example.com',
    description='Topological routing and body-relative discrete action planning',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'route_planner_node = agv_planning.route_planner_node:main',
        ],
    },
)
