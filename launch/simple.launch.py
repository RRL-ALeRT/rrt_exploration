import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="eta", default_value="1.0"),
            launch.actions.DeclareLaunchArgument(name="Geta", default_value="15.0"),
            launch_ros.actions.Node(
                package="rrt_exploration",
                executable="global_rrt_detector",
                name="global_detector",
                output="screen",
                prefix="xterm -hold -e",
                parameters=[
                    {
                        "use_sim_time": True,
                        "namespace_init_count": 1,
                        "eta": launch.substitutions.LaunchConfiguration("Geta"),
                        "map_topic": "/map",
                        # "inflation_windows_size": 15,
                    },
                    os.path.join(
                        get_package_share_directory("rrt_exploration"),
                        "config",
                        "global_rrt_detector.yaml",
                    ),
                ],
            ),
            launch_ros.actions.Node(
                package="rrt_exploration",
                executable="local_rrt_detector",
                name="local_detector",
                output="screen",
                prefix="xterm -hold -e",
                parameters=[
                    {
                        "use_sim_time": True,
                        "namespace_init_count": 1,
                        "eta": launch.substitutions.LaunchConfiguration("eta"),
                        "map_topic": "/map",
                        "robot_frame": "/base_link",
                        # "inflation_windows_size": 15,
                    },
                    os.path.join(
                        get_package_share_directory("rrt_exploration"),
                        "config",
                        "local_rrt_detector.yaml",
                    ),
                ],
            ),
            launch_ros.actions.Node(
                package="rrt_exploration",
                executable="filter.py",
                name="filter",
                output="screen",
                prefix="xterm -hold -e",
                parameters=[
                    {
                        "use_sim_time": True,
                        # 'namespace_init_count': 1,
                        # 'map_topic': '/map',
                        "info_radius": 6.0,
                        # 'costmap_clearing_threshold': 70,
                        # 'goals_topic': '/detected_points',
                        # 'namespace': '',
                        # 'n_robots': 1,
                        # 'rate': 50
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="rrt_exploration",
                executable="assigner.py",
                name="assigner",
                output="screen",
                prefix="xterm -hold -e",
                parameters=[
                    {
                        "use_sim_time": True,
                        # 'namespace_init_count': 1,
                        # 'map_topic': '/map',
                        # # 'global_frame': '/map',
                        "info_radius": 6.0,
                        # 'info_multiplier': 3.0,
                        # 'hysteresis_radius': 3.0,
                        "hysteresis_gain": 1.2,
                        # 'frontiers_topic': '/filtered_points',
                        # 'n_robots': 1,
                        # 'namespace': '',
                        # 'delay_after_assignement': 0.1,
                        # 'rate': 50
                    }
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
