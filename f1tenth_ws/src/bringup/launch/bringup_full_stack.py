# MIT License


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():

    return LaunchDescription(
        [
            safety_node = Node(
                package="safety_node", executable="safety_node.py", name="safety_node"
            ),

            gap_finder = Node(
                package="gap_finder", executable="gap_finder_base.py", name="gap_finder"
            ),

            wall_follow = Node(
                package="wall_follow", executable="wall_follow.py", name="wall_follow"
            ),

            time_to_collision = Node(
                package="time_to_collision", executable="ttc_base.py", name="ttc"
            ),

            demux = Node(package="demux", executable="demux.py", name="demux"),
            # Other actions or nodes...
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            "../../f1tenth_system/f1tenth_stack/launch/",
                            "bringup_launch.py",
                        )
                    ]
                )
            ),
        ]
    )