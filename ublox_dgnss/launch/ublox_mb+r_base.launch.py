""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""

  device_serial_string = LaunchConfiguration('device_serial_string')

  log_level_arg = DeclareLaunchArgument(
    "log_level", default_value=TextSubstitution(text="INFO")
  )
  device_serial_string_arg = DeclareLaunchArgument(
    "device_serial_string",
    default_value="Test Base",
    description="Serial string of the device to use"
  )

  params_base= [
            {'DEVICE_SERIAL_STRING': device_serial_string},
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
            {'CFG_UART2_BAUDRATE': 0x70800},

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
            {'CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2': 0x1},
            {'CFG_MSGOUT_RTCM_3X_TYPE1074_UART2': 0x1},
            {'CFG_MSGOUT_RTCM_3X_TYPE1084_UART2': 0x1},
            {'CFG_MSGOUT_RTCM_3X_TYPE1094_UART2': 0x1},
            {'CFG_MSGOUT_RTCM_3X_TYPE1124_UART2': 0x1},
            {'CFG_MSGOUT_RTCM_3X_TYPE1230_UART2': 0x1},

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
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
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
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
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
    log_level_arg,
    device_serial_string_arg,
    container_base,
    container_navsatfix,
    ])
