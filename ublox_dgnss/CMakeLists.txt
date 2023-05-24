cmake_minimum_required(VERSION 3.5)
project(ublox_dgnss)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(ublox_ubx_interfaces REQUIRED)
find_package(ublox_dgnss_node REQUIRED)
find_package(ublox_nav_sat_fix_hp_node REQUIRED)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
