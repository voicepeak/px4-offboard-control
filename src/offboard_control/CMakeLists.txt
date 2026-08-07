cmake_minimum_required(VERSION 3.0.2)
project(offboard_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    mavros_msgs
    tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_position src/offboard_position.cpp)
target_link_libraries(offboard_position ${catkin_LIBRARIES})

add_executable(offboard_pd_controller src/offboard_pd_controller.cpp)
target_link_libraries(offboard_pd_controller ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/offboard_position.py
  scripts/offboard_pd_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  offboard_position
  offboard_pd_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)