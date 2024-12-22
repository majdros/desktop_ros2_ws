# source: https://automaticaddison.com/how-to-load-a-robot-model-sdf-format-into-gazebo-ros-2/

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  gazebo_models_path = 'models'
  package_name = 'my_robot_description'
  robot_name_in_model = 'circo'
  sdf_model_path = 'models/model1.sdf'

  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.0'


  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  sdf_model_path = os.path.join(pkg_share, sdf_model_path)

  headless = LaunchConfiguration('headless')
  sdf_model = LaunchConfiguration('sdf_model')
  use_simulator = LaunchConfiguration('use_simulator')

  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')

  declare_sdf_model_path_cmd = DeclareLaunchArgument(
    name='sdf_model', 
    default_value=sdf_model_path, 
    description='Absolute path to robot sdf file')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  gazebo_params_file = os.path.join(
                  get_package_share_directory('my_robot_description'), 'config', 'gazebo_params.yaml')

  gazebo = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                  launch_arguments={'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items()
  )

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')))


  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-file', sdf_model,
                  '-x', spawn_x_val,
                  '-y', spawn_y_val,
                  '-z', spawn_z_val,
                  '-Y', spawn_yaw_val],
                  output='screen')

  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_sdf_model_path_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(gazebo)

  ExecuteProcess(
      cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
      output='screen')

  return ld