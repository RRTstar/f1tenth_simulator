cmake_minimum_required(VERSION 3.5)
project(f1tenth_simulator)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3)

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  ackermann_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  interactive_markers
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

file(GLOB SRC_FILES src/*.cpp)

add_library(${PROJECT_NAME} ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

set(LIBS ${LIBS} ${PROJECT_NAME})

# Install the library to CMAKE_INSTALL_PREFIX
# which defaults to /usr/local
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

include_directories(include ${Boost_INCLUDE_DIRS})

# Add the nodes
file(GLOB NODE_SRC_FILES node/*.cpp)
foreach(_node_file ${NODE_SRC_FILES})
    get_filename_component(_node_name ${_node_file} NAME_WE)
    add_executable(${_node_name} ${_node_file})
    target_link_libraries(${_node_name} ${LIBS})
    # add_dependencies(${_node_name} f1tenth_simulator_rosidl_generate_interfaces_cpp)
endforeach()

# Install the library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(message_runtime)

ament_package ()
