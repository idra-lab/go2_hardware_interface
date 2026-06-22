cmake_minimum_required(VERSION 3.0.2)
project(go2_hardware_interface)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  controller_manager
  realtime_tools
  base_hardware_interface
)
      
add_subdirectory(go2_hal)

catkin_package(
 CATKIN_DEPENDS
   roscpp 
   std_msgs 
   sensor_msgs 
   controller_manager
   realtime_tools
   base_hardware_interface
)

##########
# Build ##
##########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_control_node
    src/go2_robot_hw.cpp
    src/go2_ros_control.cpp
    src/go2_ros_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(ros_control_node
  ${catkin_LIBRARIES}
  go2hal
)


 
#############
## Install ##
#############

install(  TARGETS ros_control_node  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


