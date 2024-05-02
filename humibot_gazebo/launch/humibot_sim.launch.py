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
   humibot_teleop_pkg = get_package_share_directory("humibot_teleop")
   humibot_gazebo_pkg = get_package_share_directory("humibot_gazebo")
   humibot_bringup_pkg = get_package_share_directory("humibot_bringup")
   gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
   
   world_path = LaunchConfiguration("world")
   declare_world_path = DeclareLaunchArgument(
      name="world",
      default_value=os.path.join(humibot_gazebo_pkg, "worlds/sim_world.world"),
      description="Path to your gazebo world file"
   )
   
   rviz_config_file = LaunchConfiguration("rviz_config")
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config",
      default_value=os.path.join(humibot_description_pkg, "rviz/real.rviz"),
      description="Full file path for rviz2 config file"
   )
   
   use_rviz = LaunchConfiguration("use_rviz")
   declare_use_rviz = DeclareLaunchArgument(
      name="use_rviz",
      default_value="False",
      description="Start rviz2 if True"
   )
   
   use_ros2_control=LaunchConfiguration("use_ros2_control")
   declare_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control",
      default_value="True",
      description="Use ros2_control if True"
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
      description="Set spawn position in y meters"
   )
   
   controller_type = LaunchConfiguration("controller_type")
   declare_controller_type = DeclareLaunchArgument(
      name="controller_type",
      default_value="dobe",
      description="Declare what type/brand is the controller"
   )

   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(humibot_description_pkg, "launch/rsp.launch.py")]),
      launch_arguments={"use_sim_time": "True", "use_ros2_control": use_ros2_control}.items()
   )
   
   twist_mux_params= os.path.join(humibot_teleop_pkg, "config/twist_mux_params.yaml")
   start_twist_mux = Node(
      package="twist_mux",
      executable="twist_mux",
      parameters=[twist_mux_params, {"use_sim_time": True}],
      remappings=[
         ("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")
      ]
   )
   
   start_joystick = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(humibot_teleop_pkg, "launch/joystick.launch.py")]),
      launch_arguments={"use_sim_time": "True", "controller_type": controller_type}.items()
   )
   
   gazebo_params_file = os.path.join(humibot_gazebo_pkg, "config/gazebo_params.yaml")
   start_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(gazebo_ros_pkg, "launch/gazebo.launch.py")]),
      launch_arguments={"world": world_path, "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file}.items()
   )
   
   spawn_robot = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      name="spawn_entity",
      output="screen",
      arguments=[
         "-topic", "robot_description",
         "-entity", "humibot",
         "-x", x_pose,
         "-y", y_pose
      ]
   )
   
   # start controllers
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

   # start rviz
   start_rviz = Node(
      condition=IfCondition(use_rviz),
      package="rviz2",
      executable="rviz2",
      arguments=["-d", rviz_config_file],
      parameters=[
         {"use_sim_time": True}
      ]
   )
   
   return LaunchDescription([
      declare_world_path,
      declare_rviz_config_file,
      declare_use_rviz,
      declare_ros2_control,
      declare_x_pose,
      declare_y_pose,
      declare_controller_type,
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_rviz]
         )
      ),
      
      start_robot_state_publisher,
      start_twist_mux,
      start_joystick,
      start_gazebo,
      spawn_robot,
      start_joint_broadcaster,
   ])