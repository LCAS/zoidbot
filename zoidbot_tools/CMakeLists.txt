cmake_minimum_required(VERSION 2.8.3)
project(zoidbot_tools)

find_package(catkin REQUIRED COMPONENTS rospy baxter_interface)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zoidbot_tools
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/demo.py
   scripts/start.sh
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## TODO
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_zoidbot_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
