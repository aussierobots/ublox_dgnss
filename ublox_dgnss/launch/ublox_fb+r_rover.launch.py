"""
Launch a ublox_dgnss_node and a ublox_nav_sat_fix_hp_node for the rover in the
fixed base and moving rover use case.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    ublox_dgnss_dir = get_package_share_directory("ublox_dgnss")
    launch_dir = os.path.join(ublox_dgnss_dir, "launch")

    log_level = LaunchConfiguration("log_level")
    namespace = LaunchConfiguration("namespace")
    device_serial_string = LaunchConfiguration("device_serial_string")
    frame_id = LaunchConfiguration("frame_id")

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="INFO")
    )
    namespace_arg = DeclareLaunchArgument("namespace", default_value="rover")
    device_serial_string_arg = DeclareLaunchArgument(
        "device_serial_string",
        default_value="",
        description="Serial string of the device to use",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="rover",
        description="The frame_id to use in header of published messages",
    )

    rover_hpposllh_navsatfix_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "ublox_rover_hpposllh_navsatfix.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "device_serial_string": device_serial_string,
            "frame_id": frame_id,
        }.items(),
    )

    return LaunchDescription(
        [
            log_level_arg,
            namespace_arg,
            device_serial_string_arg,
            frame_id_arg,
            rover_hpposllh_navsatfix_launch_file,
        ]
    )
