cmake_minimum_required(VERSION 3.8)
project(skiros2_pyrobosim_lib)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/yaml_world_to_turtle.py
        scripts/update_battery_percentage.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    owl
    planning
    scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
