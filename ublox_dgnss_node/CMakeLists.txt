cmake_minimum_required(VERSION 3.8)

# Set policies to suppress warnings
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # For DOWNLOAD_EXTRACT_TIMESTAMP
endif()

# Suppress deprecation warnings from dependencies
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0054 NEW)

# Configure to ignore certain warnings
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

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

# Configure RPATH settings for the project
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

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

# Add nlohmann/json dependency
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

include_directories(include SYSTEM)

add_library(ublox_dgnss_components SHARED
  src/usb.cpp
  src/ublox_dgnss_node.cpp
  src/ubx/ubx.cpp
  src/ubx/cfg/ubx_cfg_item.cpp
  src/ubx/cfg/ubx_cfg_parameter.cpp
  src/ubx/cfg/ubx_cfg_parameter_loader.cpp
  src/ubx/cfg/ubx_cfg_handler.cpp
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
  nlohmann_json::nlohmann_json
)

# Set RPATH for finding shared libraries at runtime
set_target_properties(ublox_dgnss_components PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
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
  # Enable linting tests and our new minimal tests
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  # Set to true for linters we want to run
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  
  # Common test settings
  set(TEST_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ublox_ubx_msgs_INCLUDE_DIRS}
  )
  
  set(TEST_LIBRARIES
    ublox_dgnss_components
    ${ublox_ubx_msgs_LIBRARIES}
  )
  
  # Common RPATH settings for all test targets
  set(TEST_RPATH_PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;${CMAKE_INSTALL_PREFIX}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  
  # Add our new transceiver test
  ament_add_gmock(test_ubx_transceiver
    test/ubx/test_ubx_transceiver.cpp
    src/ubx/mock_ubx_transceiver.cpp
  )
  target_link_libraries(test_ubx_transceiver ${TEST_LIBRARIES})
  target_include_directories(test_ubx_transceiver PUBLIC ${TEST_INCLUDE_DIRS})
  set_target_properties(test_ubx_transceiver PROPERTIES ${TEST_RPATH_PROPERTIES})
  
  # Run all linters (except those turned off above)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
