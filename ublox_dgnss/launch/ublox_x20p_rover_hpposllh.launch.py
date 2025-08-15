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
  device_family_arg = DeclareLaunchArgument(
    "device_family", default_value=TextSubstitution(text="X20P")
  )

  params = [{'DEVICE_FAMILY': LaunchConfiguration("device_family")},
            {'CFG_USBOUTPROT_NMEA': False},
            {'CFG_RATE_MEAS': 40},
            {'CFG_RATE_NAV': 1},
            # USB output (for 0x01ab CDC-ACM interface)
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
            {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1},
            # # UART1 output (for 0x050c vendor-specific interface)
            # {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1': 1},
            # {'CFG_MSGOUT_UBX_NAV_STATUS_UART1': 5},
            # {'CFG_MSGOUT_UBX_RXM_COR_UART1': 1},
            # # UART2 output (for 0x050d vendor-specific interface)
            # {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART2': 1},
            # {'CFG_MSGOUT_UBX_NAV_STATUS_UART2': 5},
            # {'CFG_MSGOUT_UBX_RXM_COR_UART2': 1}
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
    device_family_arg,
    container1,
    ])
