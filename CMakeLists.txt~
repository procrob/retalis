cmake_minimum_required(VERSION 2.8.3)
project(retalis)



find_package(Boost REQUIRED COMPONENTS system thread)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  actionlib
  message_generation	
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Radar.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   AddOutputSubscription.srv
   AddMemory.srv
   DeleteOutputSubscription.srv
   DeleteMemory.srv
   AddInputSubscription.srv
   DeleteInputSubscription.srv		
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs roslib actionlib message_runtime
   DEPENDS Boost
)

###########
## Build ## 
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/lib/swipl-6.6.6/include
)
link_directories(/usr/lib/swipl-6.6.6/lib/x86_64-linux)

add_library(retalis_output_interface src/retalis_output_interface.cpp)
add_library(swiglm src/swiglm.cpp)



add_executable(retalis_node src/retalis_interface.cpp)
add_dependencies(retalis_node ${catkin_EXPORTED_TARGETS})

 target_link_libraries(retalis_node
   ${catkin_LIBRARIES}
   libswipl.so    
  ${Boost_LIBRARIES}
 ) 

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS retalis retalis_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_retalis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
