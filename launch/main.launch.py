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
    load_only_solutions = str2bool(LaunchConfiguration('load_only_solutions').perform(context))
    include_current_solution = str2bool(LaunchConfiguration("include_current_solution").perform(context))

    environment_deterministic = problem_number < 3
    ### Build skill list of skills to load:
    # Add your own skills here:
    own_skills = []  # ["my_primitive", "my_skill"]

    # This is our basic skill set that we work with:
    primitive_skills = ["navigate_execution", "pick_execution", "place_execution", "open_execution", "close_execution", "wm_set_relation", "wm_move_object", "wm_set_properties", "bb_unset_param"]
    # The compound skills depend on the determinism of the environment
    if environment_deterministic:
        basic_compound_skills = ["navigate", "pick", "place", "open_openablelocation", "close_openablelocation"]
        basic_compound_solution_skills = list(basic_compound_skills)
    else:
        basic_compound_skills = ["navigate_with_retry", "pick_with_retry", "place_with_retry", "open_openablelocation_with_retry", "close_openablelocation_with_retry"]
        basic_compound_solution_skills = ["pick_with_retry_solution", "place_with_retry_solution", "open_openablelocation_with_retry_solution", "close_openablelocation_with_retry_solution"]
    # Then we have new skills for each of the problems:
    problem_1_item_skills_given = ["open_location", "skip_open_location", "skip_open_openablelocation"]
    problem_2_waste_skills_given = ["select_doors_to_target", "skip_close_openablelocation", "skip_close_location", "close_location", "location_is_door", "copy_value", "is_none", "navigate_and_open_door", "navigate_and_open_doors"]
    problem_4_charge_skills_given = ["charge_directly", "charge_and_open_doors", "charger_location_from_wm", "battery_above_level", "battery_check_and_charge"]

    planner_skills = ["plan_from_file", "extract_pddl_goal_from_file", "task_plan"]
    problem_skill_list = [*basic_compound_skills]
    solution_skills = [*basic_compound_solution_skills]
    skill_list = [*primitive_skills, *planner_skills, *own_skills]

    if problem_number > 0:
        skill_list.extend(problem_1_item_skills_given)
        if problem_number > 1 or include_current_solution:
            problem_skill_list.append("problem_1")
            solution_skills.append("problem_1_solution")
    if problem_number > 1:
        skill_list.extend(problem_2_waste_skills_given)
        if problem_number > 2 or include_current_solution:
            problem_skill_list.append("problem_2")
            solution_skills.append("problem_2_solution")
    if problem_number > 2:
        if problem_number > 3 or include_current_solution:
            problem_skill_list.append("problem_3")
            solution_skills.append("problem_3_solution")
    if problem_number > 3:
        skill_list.extend(problem_4_charge_skills_given)
        if problem_number > 4 or include_current_solution:
            problem_skill_list.append("problem_4")
            solution_skills.append("problem_4_solution")

    if problem_number == 3:
        solution_skills.append("navigate_with_retry_solution")
    if problem_number == 4:
        problem_skill_list.append("navigate_with_retry_and_battery_check")
        solution_skills.append("navigate_with_retry_and_battery_check_solution")

    if load_only_solutions:
        skill_list.extend(solution_skills)
    else:
        skill_list.extend(problem_skill_list)

    ### SkiROS2 configuration and launch setup
    skiros_config = {
        "libraries_list": "[skiros2_pyrobosim_lib, skiros2_std_skills]",
        "skill_list": f"[{','.join(skill_list)}]",
        "init_scene": f"p{problem_number}_scene.turtle",
        "verbose": LaunchConfiguration('verbose').perform(context),
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
    start_pyrobosim = LaunchConfiguration('start_pyrobosim')

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "problem_number",
            default_value="1",
            description="The problem to start",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "load_only_solutions",
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
    ld.add_action(
        DeclareLaunchArgument(
            "verbose",
            default_value="False",
            description="Start SkiROS2 in verbose mode",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "include_current_solution",
            default_value="True",
            description="Decides if current solution should be loaded. Previous solutions will still be loaded.",
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


def str2bool(bool_string: str) -> bool:
    assert bool_string.lower() in ["true", "false"], f"Expected bool as string, got '{bool_string}'"
    return bool_string.lower() == "true"