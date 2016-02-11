cmake_minimum_required(VERSION 2.8.3)
project(people_tracker_filter)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
  geometry_msgs
  visualization_msgs
  bayes_people_tracker
  people_msgs
  message_filters
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(people_tracker_filter src/filter.cpp)

add_dependencies(people_tracker_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(people_tracker_filter
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS people_tracker_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

