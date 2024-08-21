import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    skiros_config = {
        "libraries_list": "['skiros2_pyrobosim_lib']",
        "skill_list": "[my_primitive, my_skill]",
        "init_scene": "''",
        "verbose": "true",
        "workspace_dir": get_package_share_directory('skiros2_pyrobosim_lib') + "/owl",
        "robot_name": "robi_robot",
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
