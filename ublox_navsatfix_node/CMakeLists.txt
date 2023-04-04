cmake_minimum_required(VERSION 3.8)
project(trot_navsatfix_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(ublox_ubx_interfaces REQUIRED)
# find_package(ublox_dgnss REQUIRED)


# include directories
include_directories(include)

# add executable
add_executable(trot_navsatfix_node src/trot_navsatfix_node.cpp)

ament_target_dependencies(trot_navsatfix_node 
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs 
  ublox_ubx_msgs
  ublox_ubx_interfaces  
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

# link libraries
target_link_libraries(trot_navsatfix_node ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES})

# install executable
install(TARGETS trot_navsatfix_node
        DESTINATION lib/${PROJECT_NAME})


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
