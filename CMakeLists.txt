cmake_minimum_required(VERSION 3.5)
project(nmea2fix)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

# add_library(nmea2fix
#   src/nmea2fix_core.cpp
# )


add_executable(nmea2fix src/nmea2fix_node.cpp src/nmea2fix_core.cpp)
ament_target_dependencies(nmea2fix rclcpp sensor_msgs nmea_msgs)

install(TARGETS nmea2fix
DESTINATION lib/${PROJECT_NAME})

# install(TARGETS launch
# DESTINATION install/${PROJECT_NAME})

ament_package()
