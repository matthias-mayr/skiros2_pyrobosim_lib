import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    problem_number = LaunchConfiguration('problem_number')
    start_pyrobosim = LaunchConfiguration('start_pyrobosim')

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "problem_number",
            default_value="3",
            description="The problem to start",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "start_pyrobosim",
            default_value="True",
            description="Start pyrobosim world as well",
        )
    )
    skiros_config = {
        "libraries_list": "[skiros2_pyrobosim_lib, skiros2_std_skills]",
        "skill_list": TextSubstitution(text=f"[navigate, pick, place, open_openablelocation, close_openablelocation, open_hallway_door, close_hallway_door, charge,\
                            navigate_execution, pick_execution, place_execution, open_execution, close_execution, get_battery_percentage,\
                            wm_set_relation, wm_move_object, wm_set_properties, bb_unset_param, success,\
                            select_object_to_fetch, skip_open_openablelocation, skip_open_location, open_location, skip_close_openablelocation, skip_close_location, close_location,\
                            problem_1_solution, problem_2_solution, problem_3_solution, move_all_objects]"),
        "init_scene": [TextSubstitution(text="p"), problem_number, TextSubstitution(text="_scene.turtle")],
        "verbose": "false",
        "workspace_dir": get_package_share_directory("skiros2_pyrobosim_lib") + "/owl",
        "robot_name": "robot",
        "robot_ontology_prefix": "robi",
    }

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("skiros2"), "skiros2.launch.py")),
            launch_arguments=skiros_config.items(),
        )
    )

    ld.add_action(
        Node(
            condition=IfCondition(start_pyrobosim),
            package="delib_ws_worlds",
            executable="run",
            name="pyrobosim_world",
            parameters=[{"problem_number": problem_number}],
        ),
    )

    return ld
