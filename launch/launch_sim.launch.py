import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
  package_name='mole'

  rsp = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
          )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
  )

  slam = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','online_async_launch.py'
          )])
  )

  default_world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'empty.world'
  )    
  
  world = LaunchConfiguration('world')

  world_arg = DeclareLaunchArgument(
    'world',
    default_value=default_world,
    description='World to load'
  )

  # Include the Gazebo launch file, provided by the ros_gz_sim package
  gazebo = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                  launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
            )

  # Run the spawner node from the ros_gz_sim package
  spawn_entity = Node(package='ros_gz_sim', executable='create',
                      arguments=['-topic', '/robot_description',
                                  '-name', 'mole',
                                  '-z', '0.2'],
                      output='screen')


  diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_controller"],
  )

  joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broadcaster"],
  )


  bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
  ros_gz_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}',
    ]
  )

  twist_stamper = Node(
    package="twist_stamper",
    executable="twist_stamper",
    parameters=[{"use_sim_time": True}],
    remappings=[("/cmd_vel_in", "/cmd_vel"),
                ("/cmd_vel_out", "/diff_controller/cmd_vel")]
  )

  # Launch
  return LaunchDescription([
    rsp,
    world_arg,
    gazebo,
    ros_gz_bridge,
    spawn_entity,
    diff_drive_spawner,
    joint_broad_spawner,
    twist_stamper,
    slam,
  ])