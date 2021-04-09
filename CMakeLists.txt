cmake_minimum_required(VERSION 2.8.3)
project(nmea_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nmea_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   std_msgs
   nmea_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nmea_tcp src/nmea_tcp.cpp)
target_link_libraries(nmea_tcp ${catkin_LIBRARIES})
add_dependencies(nmea_tcp ${catkin_EXPORTED_TARGETS})

add_executable(nmea_udp src/nmea_udp.cpp)
target_link_libraries(nmea_udp ${catkin_LIBRARIES})
add_dependencies(nmea_udp ${catkin_EXPORTED_TARGETS})

add_executable(nmea_serial src/nmea_serial.cpp)
target_link_libraries(nmea_serial ${catkin_LIBRARIES})
add_dependencies(nmea_serial ${catkin_EXPORTED_TARGETS})
