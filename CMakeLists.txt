cmake_minimum_required(VERSION 3.0.2)
project(tresc3_scan_algorithm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tresc3_scan_node src/tresc3_scan_node.cpp)
target_link_libraries(tresc3_scan_node ${catkin_LIBRARIES})
add_dependencies(tresc3_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
