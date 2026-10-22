# CMakeLists.txt for the awm package.
#
# catkin (the ROS build system) reads this file. It declares which packages
# we depend on, which messages we define, which programs we build, and where
# to install them. Sections are numbered below.

# Minimum CMake version. ROS Noetic ships CMake 3.16, so this is always met.
cmake_minimum_required(VERSION 3.0.2)

# Name of this package. It must match <name> in package.xml, and it is the
# namespace of the messages we generate (for example awm/GoalPoints.h).
project(awm)

# ROS Noetic targets C++14.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 1. Build dependencies
# ---------------------------------------------------------------------------
# catkin finds each ROS package listed here and sets two variables:
#   ${catkin_INCLUDE_DIRS}  where the headers are
#   ${catkin_LIBRARIES}     the libraries to link against
# Every ROS package whose headers we #include must appear here AND in
# package.xml. Missing it here means the compiler cannot find the header.
find_package(catkin REQUIRED COMPONENTS
  actionlib            # actionlib::SimpleActionClient (goals for move_base)
  geometry_msgs        # geometry_msgs/PoseStamped.h
  message_generation   # turns the files in srv/ into C++ headers
  move_base_msgs       # move_base_msgs/MoveBaseAction.h
  nav_msgs             # nav_msgs/OccupancyGrid.h (the map)
  roscpp               # ros/ros.h: ros::init, ros::Publisher, ros::spinOnce, ...
  tf2                  # tf2/LinearMath/Quaternion.h
  tf2_geometry_msgs    # tf2::toMsg, converts quaternions to messages
  tf2_ros              # tf2_ros::Buffer and tf2_ros::TransformListener
)

# ---------------------------------------------------------------------------
# 2. Interfaces owned by this package (messages and services)
# ---------------------------------------------------------------------------
# add_service_files lists the definition files.
# generate_messages() then creates the C++ headers, such as awm/GoalPoints.h.
# It must come after the add_* calls. DEPENDENCIES lists the packages whose messages the
# definitions use (GoalPoints.srv uses geometry_msgs/Point).
add_service_files(
  FILES
  GoalPoints.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

# ---------------------------------------------------------------------------
# 3. What other packages need when they depend on awm
# ---------------------------------------------------------------------------
# CATKIN_DEPENDS lists the packages that the headers and messages of awm use.
# Other packages that depend on awm get these automatically.
# INCLUDE_DIRS and LIBRARIES export the awm_core library (headers in
# include/awm/) to packages that depend on awm.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES awm_core
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    message_runtime    # needed at run time by the generated messages
    move_base_msgs
    nav_msgs
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

# Header search paths for every target in this package.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 4. Library
# ---------------------------------------------------------------------------
# Code shared by the nodes (namespace awm). Headers in include/awm/.
add_library(awm_core
  src/lib/frontiers.cpp
  src/lib/grid.cpp
  src/lib/map_dump.cpp
  src/lib/measure_map.cpp
  src/lib/nearest_unmeasured_goal_source.cpp
  src/lib/robot_pose.cpp
  src/lib/safe_map.cpp
  src/lib/service_goal_source.cpp
)
# service_goal_source.cpp includes the generated awm/GoalPoints.h.
add_dependencies(awm_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(awm_core ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# 5. Programs
# ---------------------------------------------------------------------------
# Each name below is built from src/nodes/<name>.cpp.
set(AWM_EXECUTABLES
  brain
  map_processing
  points_identification
)

foreach(target ${AWM_EXECUTABLES})
  add_executable(${target} src/nodes/${target}.cpp)

  # Wait for the generated message headers (such as awm/GoalPoints.h)
  # before compiling this target.
  add_dependencies(${target}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  # Link the shared library and the ROS libraries.
  target_link_libraries(${target}
    awm_core
    ${catkin_LIBRARIES}
  )
endforeach()

# ---------------------------------------------------------------------------
# 6. Unit tests
# ---------------------------------------------------------------------------
# Run with "catkin_make run_tests_awm". They test pure functions of awm_core,
# so they need no roscore.
if(CATKIN_ENABLE_TESTING)
  foreach(test test_frontiers test_goal_source test_grid test_measure_map test_safe_map)
    catkin_add_gtest(${test} test/${test}.cpp)
    target_link_libraries(${test} awm_core ${catkin_LIBRARIES})
  endforeach()
endif()

# ---------------------------------------------------------------------------
# 7. Installation
# ---------------------------------------------------------------------------
# "rosrun awm <name>" and "roslaunch awm ..." find files installed here.
install(TARGETS ${AWM_EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS awm_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
