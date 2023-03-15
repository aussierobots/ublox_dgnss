""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""
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
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        parameters=params
      )
    ]
  )

  return launch.LaunchDescription([container1])
