import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    skiros_config = {
        "libraries_list": "[skiros2_pyrobosim_lib, skiros2_std_skills]",
        "skill_list": "[navigate, pick, place, open_location, close_location, open_door, close_door, charge,\
                            navigate_execution, pick_execution, place_execution, open_execution, close_execution,\
                            wm_set_relation, wm_move_object, wm_set_properties, bb_unset_param, success,\
                            problem_1_solution, problem_2_solution]",
        "init_scene": "p2_scene.turtle",
        "verbose": "true",
        "workspace_dir": get_package_share_directory('skiros2_pyrobosim_lib') + "/owl",
        "robot_name": "robot",
        "robot_ontology_prefix": "robi"
    }

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('skiros2'),
                'skiros2.launch.py')),
        launch_arguments=skiros_config.items(),
    )
     
    return LaunchDescription([launch_include])
