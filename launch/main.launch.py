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
    load_solutions = LaunchConfiguration('load_solutions').perform(context)

    environment_deterministic = True if problem_number < 3 else False
    ### Build skill list of skills to load:
    # Add your own skills here, comma separated:
    own_skills = []

    # This is our basic skill set that we work with:
    primitive_skills = ["navigate_execution", "pick_execution", "place_execution", "open_execution", "close_execution", "wm_set_relation", "wm_move_object", "wm_set_properties", "bb_unset_param"]
    if environment_deterministic:
        basic_compound_skills = ["navigate", "pick", "place", "open_openablelocation", "close_openablelocation"]
    else:
        basic_compound_skills = ["navigate_with_retry", "pick_with_retry", "place_with_retry", "open_openablelocation_with_retry", "close_openablelocation_with_retry"]
    # Then we have new skills for each of the problems and their solutions:
    problem_1_item_skills = ["problem_1", "open_location", "skip_open_location", "skip_open_openablelocation"]
    solution_skills = ["problem_1_solution"]
    problem_2_waste_skills = ["select_doors_to_target", "skip_close_openablelocation", "skip_close_location", "close_location", "open_hallway_door", "close_hallway_door", "location_is_door", "copy_value", "is_none", "success", "navigate_and_open_door", "navigate_and_open_doors"]
    problem_3_table_skills = ["problem_3"]
    problem_4_charge_skills = ["problem_4_solution", "charge", "charger_location_from_wm", "battery_above_level", "battery_check_and_charge"]
    planner_skills = ["plan_from_file", "extract_pddl_goal_from_file", "task_plan"]
    skill_list = [*primitive_skills, *basic_compound_skills, *problem_1_item_skills, *planner_skills]
    if problem_number > 1:
        skill_list.extend(problem_2_waste_skills)
        solution_skills.extend(["problem_2_solution"])
    if problem_number > 2:
        skill_list.extend(problem_3_table_skills)
        solution_skills.extend(["problem_3_solution", "navigate_with_retry_solution", "pick_with_retry_solution", "place_with_retry_solution", "open_openablelocation_with_retry_solution", "close_openablelocation_with_retry_solution"])
    if problem_number > 3:
        skill_list.extend(problem_4_charge_skills)
    skill_list.extend(own_skills)
    if load_solutions == "True":
        skill_list.extend(solution_skills)

    ### SkiROS2 configuration and launch setup
    skiros_config = {
        "libraries_list": "[skiros2_pyrobosim_lib, skiros2_std_skills]",
        "skill_list": f"[{','.join(skill_list)}]",
        "init_scene": f"p{problem_number}_scene.turtle",
        "verbose": "true",
        "workspace_dir": get_package_share_directory("skiros2_pyrobosim_lib") + "/owl",
        "robot_name": "robot",
        "robot_ontology_prefix": "robi",
    }
    skiros2_launch_configuration = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("skiros2"), "skiros2.launch.py")),
            launch_arguments=skiros_config.items(),
        )]
    # Include updater for the battery percentage for problem 4
    if problem_number > 3:
        skiros2_launch_configuration.append(Node(
            package="skiros2_pyrobosim_lib",
            executable="update_battery_percentage.py",
            name="update_battery_percentage",
            output="screen",
        ))
    return skiros2_launch_configuration


def generate_launch_description():
    problem_number = LaunchConfiguration('problem_number')
    load_solutions = LaunchConfiguration('load_solutions')
    start_pyrobosim = LaunchConfiguration('start_pyrobosim')

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "problem_number",
            default_value="4",
            description="The problem to start",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "load_solutions",
            default_value="False",
            description="Load the problem solutions as well",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "start_pyrobosim",
            default_value="True",
            description="Start pyrobosim world as well",
        )
    )

    ld.add_action(OpaqueFunction(function=skills_and_skiros2))

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
