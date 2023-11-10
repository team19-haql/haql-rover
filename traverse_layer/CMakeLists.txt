cmake_minimum_required(VERSION 3.8)
project(traverse_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# grid_map_packages()

set(dependencies
    grid_map_msgs
    grid_map_ros
    rclcpp)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
    traverse_layer_node
    src/traverse_layer_node.cpp
    src/TraverseLayer.cpp
)

ament_target_dependencies(
    traverse_layer_node SYSTEM
    ${dependencies}
)

set(targets_list
    traverse_layer_node
)

foreach(target ${targets_list})
  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${targets_list}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark other files for installation
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


#############
## Testing ##
#############

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

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_package()
