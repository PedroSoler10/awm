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

# ---------------------------------------------------------------------------
# 1. Build dependencies
# ---------------------------------------------------------------------------
# catkin finds each ROS package listed here and sets two variables:
#   ${catkin_INCLUDE_DIRS}  where the headers are
#   ${catkin_LIBRARIES}     the libraries to link against
# Every ROS package whose headers we #include must appear here AND in
# package.xml. Missing it here means the compiler cannot find the header.
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs       # actionlib_msgs/GoalStatusArray.h (move_base goal status)
  geometry_msgs        # geometry_msgs/PoseStamped.h
  message_generation   # turns the files in srv/ into C++ headers
  move_base_msgs       # move_base_msgs/MoveBaseActionResult.h
  nav_msgs             # nav_msgs/OccupancyGrid.h (the map)
  roscpp               # ros/ros.h: ros::init, ros::Publisher, ros::spinOnce, ...
  tf2                  # tf2/LinearMath/Quaternion.h
  tf2_geometry_msgs    # tf2::toMsg, converts quaternions to messages
  tf2_ros              # tf2_ros::Buffer and tf2_ros::TransformListener
)

# Boost is a C++ library, not a ROS package, so catkin does not find it.
# We use its date_time part to build the time stamps in the measure file names.
find_package(Boost REQUIRED COMPONENTS date_time)

# ---------------------------------------------------------------------------
# 2. Interfaces owned by this package (messages and services)
# ---------------------------------------------------------------------------
# add_service_files lists the definition files.
# generate_messages() then creates the C++ headers, such as awm/GoalPoints.h.
# It must come after the add_* calls.
add_service_files(
  FILES
  GoalPoints.srv
)

generate_messages()

# ---------------------------------------------------------------------------
# 3. What other packages need when they depend on awm
# ---------------------------------------------------------------------------
# CATKIN_DEPENDS lists the packages that the headers and messages of awm use.
# Other packages that depend on awm get these automatically.
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
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
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 4. Programs
# ---------------------------------------------------------------------------
# Each name below is built from src/<name>.cpp.
set(AWM_EXECUTABLES
  brain
  brainT
  map_processing
  points_identification
)

foreach(target ${AWM_EXECUTABLES})
  add_executable(${target} src/${target}.cpp)

  # Wait for the generated message headers (such as awm/GoalPoints.h)
  # before compiling this target.
  add_dependencies(${target}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  # Link the ROS libraries and the Boost library.
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endforeach()

# ---------------------------------------------------------------------------
# 5. Installation
# ---------------------------------------------------------------------------
# "rosrun awm <name>" and "roslaunch awm ..." find files installed here.
install(TARGETS ${AWM_EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
