cmake_minimum_required(VERSION 3.5.0)
project(ros_api)

find_path(PROJECT_ROOT NAMES README.md PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
message(STATUS "found PROJECT_ROOT=${PROJECT_ROOT}")

SET(AirLib_dir ${PROJECT_ROOT}/AirLib)
SET(catkin_dir ${PROJECT_ROOT}/catkin_ws)

add_compile_options(-std=c++11)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    genmsg
)

#find_package(Threads REQUIRED)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
)

###########
## Build ##
###########

include_directories(
	${AirLib_dir}/deps/eigen3
	${AirLib_dir}/include
	${catkin_INCLUDE_DIRS}
)

link_directories(
    ${AirLib_dir}/lib
    ${AirLib_dir}/deps/MavLinkCom/lib
    ${AirLib_dir}/deps/rpclib/lib
)

add_executable(
    ${PROJECT_NAME}
    src/ros_api.cpp
)

add_dependencies(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    AirLib
    MavLinkCom
    #librpc
    pthread
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_api.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
