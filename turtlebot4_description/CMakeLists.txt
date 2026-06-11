cmake_minimum_required(VERSION 3.8)
project(turtlebot4_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories to share space
install(
  DIRECTORY launch urdf meshes config worlds src savemap
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/apf_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apf_explorer
)

install(PROGRAMS
  src/mission_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_coordinator
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()