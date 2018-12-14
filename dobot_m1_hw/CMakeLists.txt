# cmake requirements
cmake_minimum_required (VERSION 2.8)

project(dobot_m1_hw)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  sensor_msgs
  rosparam_shortcuts 
  roslib
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    sensor_msgs
    rosparam_shortcuts
)

# Folders
#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT . ABSOLUTE CACHE INTERNAL "Path prefix for the project")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_ROOT}/include/dobot_m1_hw
  ${PROJECT_ROOT}/include/DobotDll
)

LINK_DIRECTORIES(
  ${PROJECT_ROOT}/lib/DobotDll_x64
)

SET(SOURCES 
    ${PROJECT_ROOT}/src/dobot_m1_hw_interface.cpp
    ${PROJECT_ROOT}/src/dobot_m1_hw_node.cpp 
)

SET(HEADERS 
    ${PROJECT_ROOT}/include/dobot_m1_hw/dobot_m1_hw_interface.h
)

# Hardware Interface
add_library(dobot_m1_hw_interface
  ${PROJECT_ROOT}/src/dobot_m1_hw_interface.cpp
)

target_link_libraries(dobot_m1_hw_interface
  ${catkin_LIBRARIES}
  DobotDllM1
)

# Main controlexecutable
add_executable(dobot_m1_hw_node ${SOURCES} ${HEADERS})

target_link_libraries(dobot_m1_hw_node
  dobot_m1_hw_interface
  ${catkin_LIBRARIES}
  DobotDllM1
)

SET_TARGET_PROPERTIES(dobot_m1_hw_node PROPERTIES COMPILE_FLAGS "-std=c++11 -fpermissive -Wno-write-strings -Wno-format -Wno-int-to-pointer-cast -Wno-unused-result") 
TARGET_LINK_LIBRARIES(dobot_m1_hw_node ${catkin_LIBRARIES} pthread DobotDllM1)   

# Install libraries
install(TARGETS
  dobot_m1_hw_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
  dobot_m1_hw_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY 
  include/dobot_m1_hw/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
