from launch import LaunchDescription
from launch_ros.actions import Node

import yaml
import os

from moveit_configs_utils import MoveItConfigsBuilder
from yaml.loader import SafeLoader

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    bringup_config_path = os.path.join(get_package_share_directory('pm_robot_bringup'), 'config/pm_robot_bringup_config.yaml')
    
    f = open(bringup_config_path)
    bringup_config = yaml.load(f,Loader=SafeLoader)
    f.close()
    
    # Specify the name of the package and path to xacro file within the package
    file_subpath = 'urdf/pm_robot_main.xacro'

    # Use xacro to process the file
    pm_main_xacro_file = os.path.join(get_package_share_directory('pm_robot_description'), file_subpath)
    
    sim_time = True

    mappings={
        'launch_mode': 'sim_HW',
        'with_Tool_MPG_10': str(bringup_config['pm_robot_tools']['MPG_10']['with_Tool_MPG_10']),
        'with_Gonio_Left': str(bringup_config['pm_robot_gonio_left']['with_Gonio_Left']),
        'with_Gonio_Right': str(bringup_config['pm_robot_gonio_right']['with_Gonio_Right']),
        'with_Tool_MPG_10_Jaw_3mm_Lens': str(bringup_config['pm_robot_tools']['MPG_10']['Config']['with_Tool_MPG_10_Jaw_3mm_Lens']),
        'with_Tool_SPT_Holder': str(bringup_config['pm_robot_tools']['SPT_Tool_Holder']['with_Tool_SPT_Holder']),
        'with_SPT_R_A1000_I500': str(bringup_config['pm_robot_tools']['SPT_Tool_Holder']['Config']['with_SPT_R_A1000_I500']),
    }


    moveit_config = (
        MoveItConfigsBuilder("pm_robot", package_name="pm_robot_moveit_config")
        .robot_description(file_path=pm_main_xacro_file,mappings=mappings)
        .robot_description_semantic(file_path="config/pm_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    pm_moveit_server = Node(
        package="pm_moveit_server",
        executable="pm_moveit_server",
        name="pm_moveit_server",
        #output="log",
        parameters=[
            {"use_sim_time": sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        emulate_tty=True
    )

    # framework = Node(
    #         package='programming_fw_pkg',           
    #         executable='new_python_gui',      
    # )

    # subscriber_node = Node(
    #     package="programming_fw_pkg",
    #     executable="subscriber_pattern_script"
    # )

    #ld.add_action(framework)
    ld.add_action(pm_moveit_server)
    #ld.add_action(subscriber_node)

    return ld