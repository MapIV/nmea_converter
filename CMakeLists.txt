cmake_minimum_required(VERSION 2.8.3)
project(nmea_converter)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nmea_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nmea_converter
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(nmea_converter
  src/nmea_converter_core.cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

target_link_libraries(nmea_converter
  ${catkin_LIBRARIES}
)

install(TARGETS nmea_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(catkin REQUIRED COMPONENTS
  nmea_converter
)

add_executable(nmea_converter_node src/nmea_converter_node.cpp)
target_link_libraries(nmea_converter_node ${catkin_LIBRARIES} )
add_dependencies(nmea_converter_node ${catkin_EXPORTED_TARGETS})
