"""
Launch a ublox_dgnss_node for the base in the fixed base and moving rover
use case.
"""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for ublox_dgnss components."""

    log_level = LaunchConfiguration("log_level")
    namespace = LaunchConfiguration("namespace")
    device_serial_string = LaunchConfiguration("device_serial_string")
    frame_id = LaunchConfiguration("frame_id")

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="INFO")
    )
    namespace_arg = DeclareLaunchArgument("namespace", default_value="base")
    device_serial_string_arg = DeclareLaunchArgument(
        "device_serial_string",
        default_value="",
        description="Serial string of the device to use",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base",
        description="The frame_id to use in header of published messages",
    )

    params_base = [
        {"DEVICE_SERIAL_STRING": device_serial_string},
        {"FRAME_ID": frame_id},
        {"CFG_USBOUTPROT_NMEA": False},
        # output RTCM messages
        {"CFG_MSGOUT_RTCM_3X_TYPE1005_USB": 0x1},
        {"CFG_MSGOUT_RTCM_3X_TYPE1077_USB": 0x1},
        {"CFG_MSGOUT_RTCM_3X_TYPE1087_USB": 0x1},
        {"CFG_MSGOUT_RTCM_3X_TYPE1127_USB": 0x1},
        {"CFG_MSGOUT_RTCM_3X_TYPE1097_USB": 0x1},
        {"CFG_MSGOUT_RTCM_3X_TYPE1230_USB": 0x1},
        # set GPS to survey in mode
        {"CFG_TMODE_MODE": 0x1},
        {"CFG_TMODE_SVIN_ACC_LIMIT": 0xC350},
        {"CFG_TMODE_SVIN_MIN_DUR": 0x3C},
        # output some recommended UBX messages
        {"CFG_MSGOUT_UBX_NAV_SIG_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_PVT_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_POSLLH_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_RELPOSNED_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_STATUS_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_SVIN_USB": 0x1},
    ]

    container_base = ComposableNodeContainer(
        name="ublox_dgnss_base",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=params_base,
            )
        ],
    )

    return launch.LaunchDescription(
        [
            log_level_arg,
            namespace_arg,
            device_serial_string_arg,
            frame_id_arg,
            container_base,
        ]
    )
