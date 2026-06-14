#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
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

    rviz_arg = DeclareLaunchArgument("rviz_file", default_value="False", description="Load RVIZ file.")
    
    robot_description_config_path = (
        os.path.join(
            get_package_share_directory("elfin10_l_ros2_gazebo"),
            "urdf",
            "elfin10_l.urdf.xacro",
        )
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', robot_description_config_path,
        ' use_fake_hardware:=false',
        ' use_real_hardware:=true',])

    robot_description = {'robot_description': robot_description_config}

    # load robot description, srdf
    robot_description_semantic_config = load_file(
        "elfin10_l_ros2_moveit2", "config/elfin10_l.srdf"
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # load kinematics.yaml
    kinematics_yaml = load_yaml(
        "elfin10_l_ros2_moveit2", "config/kinematics.yaml"
    )

    # planning functionality (required for RViz MoveIt plugin)
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

    # Rviz
    load_rviz = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("elfin10_l_ros2_moveit2"), "launch")
    rviz_full_config = os.path.join(rviz_base, "elfin10_l_moveit2.rviz")
    
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(load_rviz),
    )

    return LaunchDescription(
        [   
            rviz_arg,
            rviz_node_full
        ]
    )