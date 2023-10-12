""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""
  params_rover = [
            {'DEVICE_SERIAL_STRING': "Test Rover"},
            {'FRAME_ID': "rover"},

            # config measurement interval to 200 ms (ie 5 Hz) and nav update rate to once per measurement
            {'CFG_RATE_MEAS': 0xc8},
            {'CFG_RATE_NAV': 0x1},

            # disable all messages on UART1
            {'CFG_UART1INPROT_NMEA': False},
            {'CFG_UART1INPROT_RTCM3X': False},
            {'CFG_UART1INPROT_UBX': False},
            {'CFG_UART1OUTPROT_NMEA': False},
            {'CFG_UART1OUTPROT_RTCM3X': False},
            {'CFG_UART1OUTPROT_UBX': False},

            # set UART2 baud rate to 460800
            {'CFG-UART2-BAUDRATE': 0x70800},

            # receive RTCM messages only (from base) on UART2
            {'CFG_UART2INPROT_NMEA': False},
            {'CFG_UART2INPROT_RTCM3X': True},
            {'CFG_UART2INPROT_UBX': False},
            {'CFG_UART2OUTPROT_NMEA': False},
            {'CFG_UART2OUTPROT_RTCM3X': False},
            {'CFG_UART2OUTPROT_UBX': False},

            # send/receive UBX messages only on USB
            {'CFG_USBINPROT_NMEA': False},
            {'CFG_USBINPROT_RTCM3X': False},
            {'CFG_USBINPROT_UBX': True},
            {'CFG_USBOUTPROT_NMEA': False},
            {'CFG_USBOUTPROT_RTCM3X': False},
            {'CFG_USBOUTPROT_UBX': True},

            # messages required for navsatfix calcs by ROS node
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0x1},

            # output relative position messages
            {'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 0x1},
            ]

  container_rover = ComposableNodeContainer(
    name='ublox_dgnss_rover',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        namespace='rover',
        parameters=params_rover
      )
    ]
  )

  container_navsatfix = ComposableNodeContainer(
    name='ublox_nav_sat_fix_hp_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_nav_sat_fix_hp_node',
        plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
        namespace='rover',
        name='ublox_nav_sat_fix_hp'
      )
    ]
  )

  return launch.LaunchDescription([
    container_rover,
    container_navsatfix,
    ])
