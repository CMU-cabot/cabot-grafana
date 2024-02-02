cmake_minimum_required(VERSION 3.8)
project(cabot_influxdb_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

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


install(PROGRAMS
  ./script/client_node.py
  ./script/diagnostics_client.py
  ./script/pose_log_client.py
  ./script/test_client.py
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
