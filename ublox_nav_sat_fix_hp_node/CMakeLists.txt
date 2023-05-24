cmake_minimum_required(VERSION 3.8)
project(ublox_nav_sat_fix_hp_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)
# find_package(ament_cmake_uncrustify REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(ublox_ubx_interfaces REQUIRED)
# find_package(ublox_dgnss REQUIRED)


# include directories
include_directories(include SYSTEM)

add_library(ublox_nav_sat_fix_hp_components SHARED
  src/ublox_nav_sat_fix_hp_node.cpp
)

# target_compile_definitions(ublox_nav_sat_fix_hp_components
#   PRIVATE "UBLOX_NAV_SAT_FIX_HP_NODE_BUILDING_DLL"
# )

ament_target_dependencies(ublox_nav_sat_fix_hp_components
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  ublox_ubx_msgs
  ublox_ubx_interfaces
)

rclcpp_components_register_node(ublox_nav_sat_fix_hp_components
  PLUGIN "ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode"
  EXECUTABLE ublox_nav_sat_fix_hp)

# install executable
install(TARGETS
ublox_nav_sat_fix_hp_components
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
