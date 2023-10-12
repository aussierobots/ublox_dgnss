""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""
  params_base= [
            {'DEVICE_SERIAL_STRING': "Test Base"},
            {'FRAME_ID': "base"},

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

            # send RTCM messages only (to rover) on UART2
            {'CFG_UART2INPROT_NMEA': False},
            {'CFG_UART2INPROT_RTCM3X': False},
            {'CFG_UART2INPROT_UBX': False},
            {'CFG_UART2OUTPROT_NMEA': False},
            {'CFG_UART2OUTPROT_RTCM3X': True},
            {'CFG_UART2OUTPROT_UBX': False},

            # RTCM and UBX messages as required on USB
            {'CFG_USBINPROT_NMEA': False},
            {'CFG_USBINPROT_RTCM3X': True},
            {'CFG_USBINPROT_UBX': True},
            {'CFG_USBOUTPROT_NMEA': False},
            {'CFG_USBOUTPROT_RTCM3X': False},
            {'CFG_USBOUTPROT_UBX': True},

            # output RTCM messages required for moving base+rover mode on UART2
            {'CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2': 0x1},
            {'CFG-MSGOUT-RTCM_3X_TYPE1074_UART2': 0x1},
            {'CFG-MSGOUT-RTCM_3X_TYPE1084_UART2': 0x1},
            {'CFG-MSGOUT-RTCM_3X_TYPE1124_UART2': 0x1},
            {'CFG-MSGOUT-RTCM_3X_TYPE1230_UART2': 0x1},

            # messages required for navsatfix calcs by ROS node
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0x1},            
            ]

  container_base = ComposableNodeContainer(
    name='ublox_dgnss_moving_base',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        namespace='base',
        parameters=params_base
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
        namespace='base',
        name='ublox_nav_sat_fix_hp'
      )
    ]
  )

  return launch.LaunchDescription([
    container_base,
    container_navsatfix,
    ])
