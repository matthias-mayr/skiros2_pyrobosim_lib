import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def skills_and_skiros2(context, *args, **kwargs):
    problem_number = int(LaunchConfiguration('problem_number').perform(context))
    ### Build skill list of skills to load:
    # Add your own skills here, comma separated:
    own_skills = []

    # This is our basic skill set that we work with:
    reasoning_skills = ["location_is_door", "copy_value", "is_none"]
    primitive_skills = ["navigate_execution", "pick_execution", "place_execution", "open_execution", "close_execution", "get_battery_percentage", "wm_set_relation", "wm_move_object", "wm_set_properties", "bb_unset_param", "success"]
    basic_compound_skills = ["navigate", "pick", "place", "open_openablelocation", "close_openablelocation", "navigate_to_target", "navigate_and_plan"]
    # Then we have new skills for each of the problems:
    problem_1_item_skills = ["problem_1"]
    problem_2_waste_skills = ["problem_1_solution", "select_doors_to_target", "skip_open_openablelocation", "skip_open_location", "open_location", "skip_close_openablelocation", "skip_close_location", "close_location", "open_hallway_door", "close_hallway_door"]
    problem_3_table_skills = ["problem_2_solution"]
    problem_4_charge_skills = ["problem_3_solution", "charge"]
    skill_list = [*reasoning_skills, *basic_compound_skills, *primitive_skills, *problem_1_item_skills]
    if problem_number > 1:
        skill_list.extend(problem_2_waste_skills)
    if problem_number > 2:
        skill_list.extend(problem_3_table_skills)
    if problem_number > 3:
        skill_list.extend(problem_4_charge_skills)
    skill_list.extend(own_skills)

    skiros_config = {
        "libraries_list": "[skiros2_pyrobosim_lib, skiros2_std_skills]",
        "skill_list": f"[{','.join(skill_list)}]",
        "init_scene": f"p{problem_number}_scene.turtle",
        "verbose": "false",
        "workspace_dir": get_package_share_directory("skiros2_pyrobosim_lib") + "/owl",
        "robot_name": "robot",
        "robot_ontology_prefix": "robi",
    }
    return[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("skiros2"), "skiros2.launch.py")),
            launch_arguments=skiros_config.items(),
        )]


def generate_launch_description():
    problem_number = LaunchConfiguration('problem_number')
    start_pyrobosim = LaunchConfiguration('start_pyrobosim')

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "problem_number",
            default_value="1"   ,
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

    ld.add_action(OpaqueFunction(function = skills_and_skiros2))

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
