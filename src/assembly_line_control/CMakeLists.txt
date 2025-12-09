cmake_minimum_required(VERSION 3.10)
project(assembly_line_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(assembly_line_control
  PACKAGE_DIR ${PROJECT_NAME}
)

# Install entry point scripts manually
# ament_cmake_python doesn't preserve entry_points from setup.py
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/web_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_interface
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/arduino_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arduino_controller
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/sensor_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_controller
)

# Install templates, static files, and launch files
install(DIRECTORY templates static launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

