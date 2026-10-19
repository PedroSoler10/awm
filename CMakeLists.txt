cmake_minimum_required(VERSION 3.0.2)
project(awm)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  GoalPoints.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    move_base_msgs
    nav_msgs
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(AWM_EXECUTABLES
  brain
  brainT
  mapper
  mapperT
  map_processing
  points_identification
)

add_executable(brain src/brain.cpp)
add_executable(brainT src/brainT.cpp)
add_executable(mapper src/mapper.cpp)
add_executable(mapperT src/mapperT.cpp)
add_executable(map_processing src/map_processing.cpp)
add_executable(points_identification src/points_identification.cpp)

foreach(target ${AWM_EXECUTABLES})
  add_dependencies(${target}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(${target} ${catkin_LIBRARIES})
endforeach()

install(TARGETS ${AWM_EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
