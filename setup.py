import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'industrial_sorting_sim'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[

        # --------------------------------------------------
        # ROS 2 package index
        # --------------------------------------------------
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # --------------------------------------------------
        # package.xml
        # --------------------------------------------------
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # --------------------------------------------------
        # Launch files
        # --------------------------------------------------
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        # --------------------------------------------------
        # Controller / config files
        # --------------------------------------------------
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*')
        ),

        # --------------------------------------------------
        # Gazebo worlds
        # --------------------------------------------------
        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*')
        ),

        # --------------------------------------------------
        # URDF / Xacro
        # --------------------------------------------------
        (
            os.path.join(
                'share',
                package_name,
                'urdf'
            ),
            glob('urdf/*')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='vboxuser',
    maintainer_email='longkaiz0324@gmail.com',

    description=(
        'ROS 2 industrial sorting simulation using '
        'UR5, Gazebo, vision and Robotiq gripper.'
    ),

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [

            'color_detector = '
            'industrial_sorting_sim.color_detector:main',

            'conveyor_controller = '
            'industrial_sorting_sim.conveyor_controller:main',

            'sorting_robot_controller = '
            'industrial_sorting_sim.sorting_robot_controller:main',

            'grasp_controller = '
            'industrial_sorting_sim.grasp_controller:main',

            'joint_nudger = '
            'industrial_sorting_sim.joint_nudger:main',
        ],
    },
)
