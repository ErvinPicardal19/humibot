from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import os

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
      launch_arguments={"use_sim_time": use_sim_time, "map": map_file}.items()
   )

   start_navigation = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         os.path.join(humibot_navigation_pkg, "launch/navigation_launch.py")
      ]),
      launch_arguments={"use_sim_time": use_sim_time}.items()
   )
   
   start_init_amcl_pose = Node(
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
   
   start_rviz = Node(
      package="rviz2",
      executable="rviz2",
      arguments=[
         "-d", rviz_config_file
      ],
      parameters=[{"use_sim_time": use_sim_time}]
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_rviz_config_file,
      declare_map_file,
      declare_x_pose,
      declare_y_pose,

      start_init_amcl_pose,
      start_amcl,
      start_navigation,
      start_rviz
   ])