cmake_minimum_required(VERSION 3.0.2)
project(robot_room)
set(CMAKE_CXX_STANDARD 11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  std_msgs
  message_generation
)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
test.msg
  #Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_room
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp rospy std_msgs  message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
include
  ${catkin_INCLUDE_DIRS} 
  ${gazebo_INCLUDE_DIRS}
 
)
#  add_executable(robot_room src/robot_sub.cpp)
#  add_executable(robot_pub src/robot_pub.cpp)
#  add_executable(robot1_sub src/robot1_sub.cpp)
#  add_executable(robot1_pub src/robot1_pub.cpp)
#  add_executable(sub_pub src/sub_pub.cpp)
#  add_executable(sub_pub1 src/sub_pub1.cpp)


 #add_executable(range src/range.cpp)

 #add_executable(differ_car src/differ_car.cpp)
 add_executable(DWA_main_1  src/DWA_main_1.cpp  src/DWA.cpp src/KinematicModel.cpp )
 target_link_libraries( DWA_main_1 ${catkin_LIBRARIES})
install(TARGETS DWA_main_1  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# add_executable(DWA_main  src/DWA_main.cpp  src/DWA.cpp src/KinematicModel.cpp )
#add_executable(main_1  src/main_1.cpp  src/KinematicModel.cpp)
## Specify libraries to link a library or executable target against
#  target_link_libraries(robot_room ${catkin_LIBRARIES})
#  target_link_libraries(robot_pub ${catkin_LIBRARIES})
#  target_link_libraries(robot1_sub ${catkin_LIBRARIES})
#  target_link_libraries(robot1_pub ${catkin_LIBRARIES})
#  target_link_libraries(sub_pub ${catkin_LIBRARIES})
#  target_link_libraries(sub_pub1 ${catkin_LIBRARIES})
#  #target_link_libraries(dwa1 ${catkin_LIBRARIES})
#  target_link_libraries(range ${catkin_LIBRARIES})
 #target_link_libraries(differ_car ${catkin_LIBRARIES})
# target_link_libraries( main_1 ${catkin_LIBRARIES})
# install(TARGETS main_1  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# target_link_libraries( DWA_main ${catkin_LIBRARIES})
# install(TARGETS DWA_main  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
