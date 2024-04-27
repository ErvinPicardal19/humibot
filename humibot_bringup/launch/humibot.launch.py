from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
   
   humibot_description_pkg = get_package_share_directory("humibot_description")
   humibot_bringup_pkg = get_package_share_directory("humibot_bringup")

   
   rviz_config_file = LaunchConfiguration("rviz_config")
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config",
      default_value=os.path.join(humibot_description_pkg, "rviz/real.rviz"),
      description="Full file path for rviz2 config file"
   )
   
   use_ros2_control=LaunchConfiguration("use_ros2_control")
   declare_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control",
      default_value="True",
      description="Use ros2_control if True"
   )

   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(humibot_description_pkg, "launch/rsp.launch.py")]),
      launch_arguments={"use_sim_time": "False", "use_ros2_control": use_ros2_control}.items()
   )
   
   # start controllers
   controllers_config_file = os.path.join(humibot_bringup_pkg, "config/controllers.yaml")
   start_controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      output="screen",
      parameters=[controllers_config_file],
      remappings=[
         ("~/robot_description", "/robot_description")
      ]
   )
   
   start_joint_broadcaster = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      output="screen",
      arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"]
   )
   start_diff_controller = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
      arguments=["diff_controller", "--controller-manager", "/controller_manager"]
   )
   
   start_lidar = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         [os.path.join(humibot_bringup_pkg, "launch/lidar.launch.py")]
      )
   )
   
   return LaunchDescription([
      declare_rviz_config_file,
      declare_ros2_control,
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      start_controller_manager,
      start_robot_state_publisher,
      start_lidar,
      start_joint_broadcaster
   ])