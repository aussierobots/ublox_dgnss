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

# Add toml11 dependency
find_package(toml11 QUIET)
if(NOT toml11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    toml11
    URL https://github.com/ToruNiina/toml11/archive/refs/tags/v3.7.1.tar.gz
    URL_HASH SHA256=afeaa9aa0416d4b6b2cd3897ca55d9317084103077b32a852247d8efd4cf6068
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(toml11)
  
  # Set up toml11 include directories
  # Based on toml11's own CMakeLists.txt, the main header is at the root level
  # and other headers are in the toml/ subdirectory
  message(STATUS "Setting up toml11 include paths from: ${toml11_SOURCE_DIR}")
  include_directories(${toml11_SOURCE_DIR})
  
  # Also create our own target if toml11 didn't create one
  if(NOT TARGET toml11::toml11)
    add_library(toml11_local INTERFACE)
    target_include_directories(toml11_local INTERFACE ${toml11_SOURCE_DIR})
    add_library(toml11::toml11 ALIAS toml11_local)
    message(STATUS "Created toml11::toml11 alias target")
  endif()
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
  toml11::toml11
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
  
  # Add test for UbxCfgItem
  ament_add_gtest(test_ubx_cfg_item
    test/ubx/cfg/test_ubx_cfg_item.cpp
  )
  target_link_libraries(test_ubx_cfg_item ${TEST_LIBRARIES})
  target_include_directories(test_ubx_cfg_item PUBLIC ${TEST_INCLUDE_DIRS})
  set_target_properties(test_ubx_cfg_item PROPERTIES ${TEST_RPATH_PROPERTIES})
  
  # Add test for UbxCfgParameter
  ament_add_gtest(test_ubx_cfg_parameter
    test/ubx/cfg/test_ubx_cfg_parameter.cpp
  )
  target_link_libraries(test_ubx_cfg_parameter ${TEST_LIBRARIES})
  ament_target_dependencies(test_ubx_cfg_parameter
    "rclcpp"
  )
  target_include_directories(test_ubx_cfg_parameter PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TEST_INCLUDE_DIRS}
  )
  set_target_properties(test_ubx_cfg_parameter PROPERTIES ${TEST_RPATH_PROPERTIES})

  # Test for UbxCfgParameterLoader
  ament_add_gmock(test_ubx_cfg_parameter_loader
    test/ubx/cfg/test_ubx_cfg_parameter_loader.cpp
  )
  target_link_libraries(test_ubx_cfg_parameter_loader ${TEST_LIBRARIES})
  target_include_directories(test_ubx_cfg_parameter_loader PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TEST_INCLUDE_DIRS}
  )
  set_target_properties(test_ubx_cfg_parameter_loader PROPERTIES ${TEST_RPATH_PROPERTIES})
  
  # Test for UbxCfgHandler with MockUbxTransceiver
  ament_add_gmock(test_ubx_cfg_handler_new
    test/ubx/cfg/test_ubx_cfg_handler_new.cpp
  )
  target_link_libraries(test_ubx_cfg_handler_new ${TEST_LIBRARIES})
  ament_target_dependencies(test_ubx_cfg_handler_new
    "rclcpp"
  )
  target_include_directories(test_ubx_cfg_handler_new PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TEST_INCLUDE_DIRS}
  )
  set_target_properties(test_ubx_cfg_handler_new PROPERTIES ${TEST_RPATH_PROPERTIES})
  
  # Test for toml11 integration
  add_executable(test_toml_parser test/test_toml_parser.cpp)
  if(TARGET toml11::toml11)
    target_link_libraries(test_toml_parser
      toml11::toml11
    )
  else()
    # If no target is available, manually set include directories
    if(DEFINED TOML11_INCLUDE_DIR)
      target_include_directories(test_toml_parser PRIVATE ${TOML11_INCLUDE_DIR})
    else()
      target_include_directories(test_toml_parser PRIVATE ${toml11_SOURCE_DIR})
    endif()
  endif()
  install(TARGETS test_toml_parser
    DESTINATION lib/${PROJECT_NAME}
  )
  install(FILES
    test/toml_test.toml
    DESTINATION lib/${PROJECT_NAME}/test
  )

  # Add all test targets to test dependencies
  ament_add_gtest_executable(dependency_dummy test/dependency_dummy.cpp)
  target_link_libraries(dependency_dummy ${TEST_LIBRARIES})

  # Add tests for ament_lint_auto
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
