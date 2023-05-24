cmake_minimum_required(VERSION 3.8)
project(ntrip_client_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)
# find_package(ament_cmake_uncrustify REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(libcurl_vendor REQUIRED)

include_directories(include SYSTEM)

add_library(ntrip_client_components SHARED
  src/ntrip_client_node.cpp
)

target_compile_definitions(ntrip_client_components
  PRIVATE "NTRIP_CLIENT_NODE_BUILDING_DLL"
)

ament_target_dependencies(ntrip_client_components
  rclcpp
  rclcpp_components
  std_msgs
  rtcm_msgs
  libcurl_vendor
)

rclcpp_components_register_node(ntrip_client_components
  PLUGIN "ublox_dgnss::NTRIPClientNode"
  EXECUTABLE ntrip_client_node
)

install(TARGETS
  ntrip_client_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
