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

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)
find_package(yaml-cpp REQUIRED)

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
  PROJ
)

add_executable(client_node
  src/cabot_rclcpp_util.cpp
  src/geoutil.cpp
  src/client_node.cpp
  src/InfluxPoint.cpp
  src/InfluxDB.cpp
)

include_directories(client_node
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PROJ_INCLUDE_DIRS}
)

ament_target_dependencies(client_node
  ${dependencies}
)

target_link_libraries(client_node
  ${OpenCV_LIBRARIES}
  InfluxData::InfluxDB
  ${PROJ_LIBRARY}
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

ament_python_install_package(cabot_ui)

install(PROGRAMS
  ./script/client_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories("include/")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
