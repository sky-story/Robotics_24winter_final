from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab_quaternion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/lab_quaternion']),
    ('share/lab_quaternion', ['package.xml']),
    ('share/lab_quaternion/launch', ['launch/sort_world.launch.py']),
    ('share/lab_quaternion/worlds', ['worlds/sort_world.sdf']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ningbo',
    maintainer_email='ningbo@uw.edu',
    description='lAB 7',
    license= 'MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'pick_and_place = lab_quaternion.pick_and_place:main',
            'gen3lite_pymoveit2 = lab_quaternion.gen3lite_pymoveit2:main',
            'sort_cubes = lab_quaternion.sort_cubes:main',
            'pick_and_place_2 = lab_quaternion.pick_and_place_2:main',
            'final_pick = lab_quaternion.final_pick:main',
            'car_bonus = lab_quaternion.car_bonus:main',
            'car = lab_quaternion.car:main',
            'final_car = lab_quaternion.final_car:main',
            'car_forward = lab_quaternion.car_forward:main',
            'car_back = lab_quaternion.car_back:main',
	],
    },
)
