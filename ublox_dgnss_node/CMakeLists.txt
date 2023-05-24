cmake_minimum_required(VERSION 3.5)
project(ublox_dgnss_node)

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
find_package(ublox_ubx_msgs REQUIRED)
find_package(ublox_ubx_interfaces REQUIRED)
find_package(rtcm_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)

include_directories(include SYSTEM)

add_library(ublox_dgnss_components SHARED
  src/usb.cpp
  src/ublox_dgnss_node.cpp
)

target_compile_definitions(ublox_dgnss_components
  PRIVATE "UBLOX_DGNSS_NODE_BUILDING_DLL"
)

ament_target_dependencies(ublox_dgnss_components
  rclcpp
  rclcpp_components
  std_msgs
  ublox_ubx_msgs
  ublox_ubx_interfaces
  rtcm_msgs
)

target_link_libraries(ublox_dgnss_components
  usb-1.0
)

rclcpp_components_register_node(ublox_dgnss_components
  PLUGIN "ublox_dgnss::UbloxDGNSSNode"
  EXECUTABLE ublox_dgnss_node)

install(TARGETS
  ublox_dgnss_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
