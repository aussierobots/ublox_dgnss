""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""

  log_level_arg = DeclareLaunchArgument(
    "log_level", default_value=TextSubstitution(text="INFO")
  )

  params = [{'CFG_USBOUTPROT_NMEA': False},
            {'CFG_RATE_MEAS': 100},
            {'CFG_RATE_NAV': 1},
            {'CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
            {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1}
            ]

  container1 = ComposableNodeContainer(
    name='ublox_dgnss_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        parameters=params
      )
    ]
  )

  return launch.LaunchDescription([
    log_level_arg,
    container1,
    ])
