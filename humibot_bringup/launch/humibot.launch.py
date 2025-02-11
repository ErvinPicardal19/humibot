from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
   
   humibot_description_pkg = get_package_share_directory("humibot_description")
   humibot_bringup_pkg = get_package_share_directory("humibot_bringup")
   humibot_teleop_pkg = get_package_share_directory("humibot_teleop")
   
   use_ros2_control=LaunchConfiguration("use_ros2_control")
   declare_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control",
      default_value="True",
      description="Use ros2_control if True"
   )
   
   lidar_port=LaunchConfiguration("lidar_port")
   declare_lidar_port = DeclareLaunchArgument(
      name="lidar_port",
      default_value="/dev/LD06",
      description="Port for the lidar"
   )
   
   server_url = LaunchConfiguration("server_url")
   declare_server_url = DeclareLaunchArgument(
      name="server_url",
      default_value="http://192.168.43.95:5000",
      description="Declare the websocket server URL address"
   )

   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(humibot_description_pkg, "launch/rsp.launch.py")]),
      launch_arguments={"use_sim_time": "False", "use_ros2_control": use_ros2_control}.items()
   )
   
   twist_mux_params= os.path.join(humibot_teleop_pkg, "config/twist_mux_params.yaml")
   start_twist_mux = Node(
      package="twist_mux",
      executable="twist_mux",
      parameters=[twist_mux_params, {"use_sim_time": False}],
      remappings=[
         ("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")
      ]
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
      ),
      launch_arguments={"serial_port": lidar_port}.items()
   )
   
   # dehumidifier_node
   start_dehumidifier_service_node = Node(
      package="humibot_hardware",
      executable="dehumidifier_service_node",
   )

   # water_lvl_node
   start_water_lvl_sensor_node = Node(
      package="humibot_hardware",
      executable="water_lvl_sensor_node",
   )
   
   # WWService Node
   start_websocket_service = Node(
      package="humibot_hardware",
      executable="WSService",
      parameters=[
         {
            "url": server_url
         }
      ]
   )
   
   return LaunchDescription([
      declare_ros2_control,
      declare_lidar_port,
      declare_server_url,
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_diff_controller,
            on_exit=[start_websocket_service]
         )
      ),
      
      start_controller_manager,
      start_robot_state_publisher,
      start_twist_mux,
      start_lidar,
      start_joint_broadcaster,
      start_dehumidifier_service_node,
      start_water_lvl_sensor_node
   ])