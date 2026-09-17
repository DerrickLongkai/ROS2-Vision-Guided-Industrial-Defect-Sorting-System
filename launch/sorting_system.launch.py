
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ======================================================
    # Paths
    # ======================================================

    ur_sim_share = get_package_share_directory(
        'ur_simulation_gz'
    )

    ur_launch = os.path.join(
        ur_sim_share,
        'launch',
        'ur_sim_control.launch.py'
    )

    description_file = (
        '/home/vboxuser/ros2_sorting_ws/src/'
        'industrial_sorting_sim/urdf/'
        'ur5_with_robotiq.urdf.xacro'
    )

    controllers_file = (
        '/home/vboxuser/ros2_sorting_ws/src/'
        'industrial_sorting_sim/config/'
        'ur_controllers_sim.yaml'
    )

    world_file = (
        '/home/vboxuser/ros2_sorting_ws/src/'
        'industrial_sorting_sim/worlds/'
        'sorting_world.sdf'
    )

    # ======================================================
    # Gazebo + UR5 + Robotiq
    # ======================================================

    ur_simulation = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            ur_launch
        ),

        launch_arguments={

            'ur_type':
                'ur5',

            'launch_rviz':
                'false',

            'description_file':
                description_file,

            'controllers_file':
                controllers_file,

            'world_file':
                world_file,

            # IMPORTANT:
            # start OUR controller directly
            'initial_joint_controller':
                'joint_trajectory_controller',

            'activate_joint_controller':
                'true',

        }.items()
    )

    # ======================================================
    # Gazebo set_pose bridge
    # ======================================================

    gazebo_service_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        arguments=[

            '/world/sorting_world/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose',

            '/world/sorting_world/remove'
            '@ros_gz_interfaces/srv/DeleteEntity',

            '/world/sorting_world/create'
            '@ros_gz_interfaces/srv/SpawnEntity',
            
            '/sorting_camera/image'
            '@sensor_msgs/msg/Image'
            '@gz.msgs.Image',
        ],

        output='screen'
    )

    # ======================================================
    # Simulated grasp controller
    # ======================================================

    grasp_controller = Node(

        package='industrial_sorting_sim',

        executable='grasp_controller',

        output='screen'
    )
    
    color_detector = Node(
        package='industrial_sorting_sim',
        executable='color_detector',
        name='color_detector',
        output='screen'
    )


    conveyor_controller = Node(
        package='industrial_sorting_sim',
        executable='conveyor_controller',
        name='conveyor_controller',
        output='screen'
    )


    sorting_robot_controller = Node(
        package='industrial_sorting_sim',
        executable='sorting_robot_controller',
        name='sorting_robot_controller',
        output='screen'
    )

    # ======================================================
    # Delay bridge + grasp until Gazebo has started
    # ======================================================

    delayed_helpers = TimerAction(

        period=6.0,

        actions=[
            gazebo_service_bridge,
            grasp_controller
        ]
    )
    
    automatic_system = TimerAction(
        period=8.0,

        actions=[
            color_detector,
            conveyor_controller,
            sorting_robot_controller,
        ]
    )

    return LaunchDescription([
        ur_simulation,
        delayed_helpers,
        automatic_system,
    ])
