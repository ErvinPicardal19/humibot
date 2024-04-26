from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
from launch.event_handlers import OnProcessExit

def generate_launch_description():
   
   description_pkg = get_package_share_directory("humibot_description")
   gazebo_pkg = get_package_share_directory("humibot_gazebo")
   gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
   
   world_path = LaunchConfiguration("world")
   declare_world_path = DeclareLaunchArgument(
      name="world",
      default_value=os.path.join(gazebo_pkg, "worlds/sim_world.world"),
      description="Path to your gazebo world file"
   )
   
   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(description_pkg, "launch/rsp.launch.py")),
      launch_arguments={"use_sim_time": "True"}.items()
   )
   
   start_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, "launch/gazebo.launch.py")),
      launch_arguments={"world": world_path}.items()
   )
   
   spawn_robot = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      output="screen",
      arguments=[
         "-topic", "robot_description",
         "-entity", "humibot"
      ]
   )
   
   # start controllers
   start_joint_broadcaster = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
      arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"]
   )
   start_diff_controller = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
      arguments=["diff_controller"]
   )

   
   
   return LaunchDescription([
      declare_world_path,
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      start_robot_state_publisher,
      start_gazebo,
      spawn_robot,
      start_joint_broadcaster
   ])