cmake_minimum_required(VERSION 3.8)
project(cabot_influxdb_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cabot_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(Boost REQUIRED)
find_package(InfluxDB REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  cabot_msgs
  tf2_geometry_msgs
  OpenCV
  cv_bridge
  boost_system
  cpr
  InfluxDB
)

add_executable(client_node
  src/cabot_rclcpp_util.cpp
  src/geoutil.cpp
  src/client_node.cpp
)

target_include_directories(client_node PRIVATE include)

target_link_libraries(client_node
  rclcpp::rclcpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  cabot_msgs::cabot_msgs__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  boost_system
  cpr
  InfluxDB

)

#ament_target_dependencies(client_node
#  ${dependencies}
#)

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

# ament_python_install_package(cabot_ui)

#install(PROGRAMS
  #./script/client_node.py
  #./script/diagnostics_client.py
  #./script/pose_log_client.py
  #./script/test_client.py
  #DESTINATION lib/${PROJECT_NAME}
  #  )

install(TARGETS
  client_node
  DESTINATION lib
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories("include/")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
