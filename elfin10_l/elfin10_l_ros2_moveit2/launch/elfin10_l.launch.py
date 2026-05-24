#!/usr/bin/python3

# elfin10_l.launch.py:
# Launch file for the elfin10_l Robot GAZEBO + MoveIt!2 SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    
    # *********************** Gazebo *********************** # 
    
    # DECLARE Gazebo WORLD file:
    elfin10_l_ros2_gazebo = os.path.join(
        get_package_share_directory('elfin10_l_ros2_gazebo'),
        'worlds',
        'elfin10_l.world')
        
    # DECLARE Gazebo Params file (Fixes 10Hz MoveIt Servo Issue):
    gazebo_params_file = os.path.join(
        get_package_share_directory('elfin10_l_ros2_gazebo'),
        'config',
        'gazebo_params.yaml')

    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={
                    'world': elfin10_l_ros2_gazebo,
                    'params_file': gazebo_params_file,
                }.items(),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # elfin10_l Description file package:
    elfin10_l_description_path = os.path.join(
        get_package_share_directory('elfin10_l_ros2_gazebo'))
    # elfin10_l ROBOT urdf file path:
    xacro_file = os.path.join(elfin10_l_description_path,
                              'urdf',
                              'elfin10_l.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for elfin10_l (Gazebo setup):
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_fake_hardware:=false',
        ' use_real_hardware:=false',])
    robot_description_gazebo = {'robot_description': robot_description_config}
    
    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'elfin10_l',"-x", "0.0", "-y", "0.0", "-z", "0.1"],
                        output='screen')

    # ***** STATIC TRANSFORM ***** #
    # NODE -> Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "elfin_base_link"],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description_gazebo, {'use_sim_time': True}],
    )

    # ***** CONTROLLERS ***** #
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'elfin_arm_controller'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'elfin_servo_controller'],
        output='screen'
    )

    close_evt1 = RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
    )
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_servo_controller],
            )
    )

    # *********************** MoveIt!2 *********************** #   
    
    # Command-line argument: RVIZ file
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT *** #
    # Robot description, URDF (Fixes Hardware Mismatch):
    robot_description_config_moveit = xacro.process_file(
        xacro_file,
        mappings={
            'use_real_hardware': 'false',
            'use_fake_hardware': 'false'
        }
    )
    robot_description_moveit = {"robot_description": robot_description_config_moveit.toxml()}
    
    # Robot description, SRDF:
    robot_description_semantic_config = load_file(
        "elfin10_l_ros2_moveit2", "config/elfin10_l.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "elfin10_l_ros2_moveit2", "config/kinematics.yaml"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "elfin10_l_ros2_moveit2", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "elfin10_l_ros2_moveit2", "config/elfin_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_moveit,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("elfin10_l_ros2_moveit2"), "launch")
    rviz_full_config = os.path.join(rviz_base, "elfin10_l_moveit2.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description_moveit,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
        condition=UnlessCondition(load_RVIZfile),
    )

    servo_yaml = load_yaml(
        'elfin10_l_ros2_moveit2', 'config/elfin_servo.yaml'
    )
    servo_params = {'moveit_servo': servo_yaml}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        parameters=[
            servo_params,
            robot_description_moveit,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        # Gazebo nodes:
        gazebo, 
        spawn_entity,
        close_evt1,
        close_evt2,
        close_evt3,
        
        # ROS2_CONTROL & TF:
        static_tf,
        robot_state_publisher,
        
        # Start MoveIt!2 stack ONLY after the robot is spawned in Gazebo
        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    rviz_arg,
                    declare_use_sim_time_cmd,
                    rviz_node_full,
                    run_move_group_node,
                    servo_node
                ]
            )
        )
    ])