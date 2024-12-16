""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""

  namespace = LaunchConfiguration('namespace')
  device_serial_string = LaunchConfiguration('device_serial_string')
  frame_id = LaunchConfiguration('frame_id')

  log_level_arg = DeclareLaunchArgument(
    "log_level", default_value=TextSubstitution(text="INFO")
  )
  namespace_arg = DeclareLaunchArgument(
    "namespace", default_value=""
  )
  device_serial_string_arg = DeclareLaunchArgument(
    "device_serial_string",
    default_value="",
    description="Serial string of the device to use"
  )
  frame_id_arg = DeclareLaunchArgument(
    "frame_id",
    default_value="ubx",
    description="The frame_id to use in header of published messages"
  )

  params = [{'DEVICE_SERIAL_STRING': device_serial_string},
            {'FRAME_ID': frame_id},
            {'CFG_USBOUTPROT_NMEA': False},
            {'CFG_RATE_MEAS': 10},
            {'CFG_RATE_NAV': 100},
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
            {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1}]

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
        namespace=namespace,
        parameters=params
      )
    ]
  )

  container2 = ComposableNodeContainer(
    name='ublox_nav_sat_fix_hp_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_nav_sat_fix_hp_node',
        plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
        name='ublox_nav_sat_fix_hp',
        namespace=namespace
      )
    ]
  )

  return launch.LaunchDescription([
    log_level_arg,
    namespace_arg,
    device_serial_string_arg,
    frame_id_arg,
    container1,
    container2,
    ])
