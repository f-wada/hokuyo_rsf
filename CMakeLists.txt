cmake_minimum_required(VERSION 3.0.2)
project(hokuyo_rsf)

add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  diagnostic_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Gpzda.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    nmea_msgs
    diagnostic_msgs
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hokuyo_rsf
  src/hokuyo_rsf.cpp
  src/payload_converter.cpp
  src/spel_parser.cpp
  src/spel_publisher.cpp
)
target_compile_features(hokuyo_rsf PUBLIC cxx_std_17)
target_link_libraries(hokuyo_rsf ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(hokuyo_rsf ${catkin_EXPORTED_TARGETS})
endif()
add_dependencies(hokuyo_rsf ${PROJECT_NAME}_generate_messages_cpp)

add_executable(send_uint8_command
  src/send_uint8_command.cpp
)
target_compile_features(send_uint8_command PUBLIC cxx_std_17)
target_link_libraries(send_uint8_command ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(send_uint8_command ${catkin_EXPORTED_TARGETS})
endif()

add_executable(spel_diagnostics_publisher
  src/spel_diagnostics_publisher.cpp
)
target_compile_features(spel_diagnostics_publisher PUBLIC cxx_std_17)
target_link_libraries(spel_diagnostics_publisher ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(spel_diagnostics_publisher ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS hokuyo_rsf send_uint8_command spel_diagnostics_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
