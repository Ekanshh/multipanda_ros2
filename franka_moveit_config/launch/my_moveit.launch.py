#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    load_gripper_parameter_name = "load_gripper"
    scene_xml_parameter_name = "scene_xml"
    mj_yaml_parameter_name = "mj_yaml"

    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    scene_xml = LaunchConfiguration(scene_xml_parameter_name)
    mj_yaml = LaunchConfiguration(mj_yaml_parameter_name)

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "panda_arm_sim.urdf.xacro",
    )
    default_scene_xml_file = os.path.join(
        get_package_share_directory("franka_description"),
        "mujoco",
        "franka",
        "scene.xml",
    )
    default_mj_yaml_file = os.path.join(
        get_package_share_directory("franka_bringup"),
        "config",
        "mujoco",
        "mj_objects.yaml",
    )
    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value="true",
        description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
        "without an end-effector.",
    )
    scene_xml_arg = DeclareLaunchArgument(
        scene_xml_parameter_name,
        default_value=default_scene_xml_file,
        description="The path to the mujoco xml file that you want to load.",
    )
    mj_yaml_arg = DeclareLaunchArgument(
        mj_yaml_parameter_name,
        default_value=default_mj_yaml_file,
        description="The path to the mujoco object yaml file that you want to load.",
    )

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " arm_id:=",
            "panda",
            " hand:=",
            load_gripper,
            " scene_xml:=",
            scene_xml,
            " mj_yaml:=",
            mj_yaml,
        ]
    )

    robot_description = {"robot_description": robot_description_config}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_moveit_config"),
        "srdf",
        "panda_arm.srdf.xacro",
    )
    robot_description_semantic_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_semantic_xacro_file,
            " arm_id:=",
            "panda",
            " hand:=",
            load_gripper,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml("franka_moveit_config", "config/kinematics.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("franka_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "franka_moveit_config", "config/panda_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager"
        "/MoveItSimpleControllerManager",
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
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # remappings=[("joint_states", "franka/joint_states")],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("franka_moveit_config"), "rviz"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
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
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "franka/joint_states",
                    "/panda_gripper_sim_node/joint_states",
                ],
                "rate": 30,
            }
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_moveit_config"),
        "config",
        "panda_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[("joint_states", "franka/joint_states")],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            scene_xml_arg,
            mj_yaml_arg,
            load_gripper_arg,
            robot_state_publisher,
            joint_state_publisher,
            ros2_control_node,
            run_move_group_node,
            rviz_node,
        ]
        + load_controllers,
    )
    # Warehouse mongodb server
