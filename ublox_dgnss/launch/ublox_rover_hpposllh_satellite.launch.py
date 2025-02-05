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
            {'CFG_UART1OUTPROT_UBX': False},
            {'CFG_UART1OUTPROT_NMEA': False},
            {'CFG_UART1OUTPROT_RTCM3X': False},
            {'CFG_RATE_MEAS': 200},
            {'CFG_RATE_NAV': 2},
            {'CFG_SEC_SPOOFDET_SIM_SIG_DIS': False},
            {'CFG_SEC_JAMDET_SENSITIVITY_HI': True},
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 20},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 20},
            {'CFG_MSGOUT_UBX_NAV_SAT_USB': 20},
            {'CFG_MSGOUT_UBX_NAV_ORB_USB': 20},
            {'CFG_MSGOUT_UBX_NAV_SIG_USB': 20},
            {'CFG_MSGOUT_UBX_SEC_SIG_USB': 20},
            {'CFG_MSGOUT_UBX_SEC_SIGLOG_USB': 20},
            {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 20},
            {'CFG_MSGOUT_UBX_RXM_COR_USB': 20},
            {'CFG_MSGOUT_UBX_RXM_RAWX_USB': 20},
            {'CFG_MSGOUT_UBX_RXM_MEASX_USB': 20}]

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
