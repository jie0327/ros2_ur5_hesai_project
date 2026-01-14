import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur_app'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@todo.todo',
    description='My UR5 Project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_test = ur_app.task.move_test:main',
            'moveit_client = ur_app.task.moveit_client:main',
            'spawn_circle_objects = ur_app.task.spawn_circle_objects:main',
            'cloud_accumulator = ur_app.task.cloud_accumulator:main',
        ],
    },
)