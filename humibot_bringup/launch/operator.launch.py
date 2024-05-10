from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition

def generate_launch_description():
   
   humibot_slam_pkg = get_package_share_directory("humibot_slam")
   humibot_description_pkg = get_package_share_directory("humibot_description")
   humibot_navigation_pkg = get_package_share_directory("humibot_navigation")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use Gazebo clock if True"
   )
   
   server_url = LaunchConfiguration("server_url")
   declare_server_url = DeclareLaunchArgument(
      name="server_url",
      default_value="http://192.168.1.10:5000",
      description="Declare the websocket server URL address"
   )
   
   # mode = LaunchConfiguration("mode")
   declare_mode = DeclareLaunchArgument(
      name="mode",
      default_value="mapping",
      description="For setting SLAM for mapping/localization"
   )
   
   rviz_config_file = LaunchConfiguration("rviz_config")
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config",
      default_value=os.path.join(humibot_description_pkg, "rviz/real.rviz"),
      description="Rviz2 config file full path"
   )
   
   map_file = LaunchConfiguration("map")
   declare_map_file = DeclareLaunchArgument(
      name="map",
      default_value=os.path.join(humibot_slam_pkg, "maps/simulation_map.yaml"),
      description="Desired map file full path"
   )
   
   x_pose=LaunchConfiguration("x_pose")
   declare_x_pose = DeclareLaunchArgument(
      name="x_pose",
      default_value="0.0",
      description="Set spawn position in x meters"
   )

   y_pose=LaunchConfiguration("y_pose")
   declare_y_pose = DeclareLaunchArgument(
      name="y_pose",
      default_value="0.0",
      description="Set spawn position in y meters",
   )
   
   start_amcl = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         os.path.join(humibot_slam_pkg, "launch/localization_launch.py")
      ]),
      condition=LaunchConfigurationEquals("mode", "localization"),
      launch_arguments={"use_sim_time": use_sim_time, "map": map_file}.items()
   )
   
   start_navigation = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         os.path.join(humibot_navigation_pkg, "launch/navigation_launch.py")
      ]),
      condition=LaunchConfigurationEquals("mode", "localization"),
      launch_arguments={"use_sim_time": use_sim_time}.items()
   )
   
   start_init_amcl_pose = Node(
      condition=LaunchConfigurationEquals("mode", "localization"),
      package="humibot_slam",
      executable="set_init_amcl_pose",
      name="amcl_init_pose_publisher",
      parameters=[
         {
            "x": x_pose,
            "y": y_pose,
         }
      ]
   )
   
   start_mapping = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         os.path.join(humibot_slam_pkg, "launch/online_async_launch.py")
      ]),
      condition=LaunchConfigurationEquals("mode", "mapping"),
      launch_arguments={"use_sim_time": use_sim_time}.items()
   )
   
   start_rviz = Node(
      package="rviz2",
      executable="rviz2",
      arguments=[
         "-d", rviz_config_file
      ],
      parameters=[{"use_sim_time": use_sim_time}]
   )
   
   # dht11_node
   start_websocket_service = Node(
      package="humibot_hardware",
      executable="WSService",
      parameters=[
         {
            "url": server_url
         }
      ]
   )

   # dht11_node
   start_dht11_node = Node(
      package="humibot_hardware",
      executable="dht11_node",
   )
   
   # dht11_service
   start_dht11_service = Node(
      package="humibot_hardware",
      executable="dht11_service",
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_rviz_config_file,
      declare_map_file,
      declare_x_pose,
      declare_y_pose,
      declare_mode,
      declare_server_url,

      start_init_amcl_pose,
      start_amcl,
      start_navigation,
      start_mapping,
      start_rviz,
      # start_websocket_service,
      # start_dht11_node,
      # start_dht11_service
   ])