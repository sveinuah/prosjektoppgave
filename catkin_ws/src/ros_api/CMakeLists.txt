cmake_minimum_required(VERSION 3.5.0)
project(ros_api)

find_path(PROJECT_ROOT NAMES README.md PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
message(STATUS "found PROJECT_ROOT=${PROJECT_ROOT}")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE} ${BUILD_TYPE}")

SET(AirLib_dir ${PROJECT_ROOT}/AirLib)
SET(catkin_dir ${PROJECT_ROOT}/catkin_ws)
SET(fisheye_dir ${PROJECT_ROOT}/Fisheyecapture/FisheyeCaptureClient)
SET(LLVM_DIR ${PROJECT_ROOT}/llvm-build)
SET(RPCLIB_INCLUDES ${PROJECT_ROOT}/external/rpclib/rpclib-2.2.1)
SET(OpenCV_DIR /usr/share/OpenCV)

set(CMAKE_CC_COMPILER ${CC})
set(CMAKE_CXX_COMPILER ${CXX})

set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")

add_compile_options(-std=c++14)

#set(CMAKE_CXX_FLAGS "-std=c++14 -ggdb -Wall -Wextra \
#    -Wstrict-aliasing -Wunreachable-code -Wcast-qual \
#    -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 \
#    -Winit-self -Wmissing-include-dirs -Wswitch-default \
#    -Wold-style-cast -Woverloaded-virtual -Wredundant-decls \
#    -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef \
#    -Wno-variadic-macros -Wno-parentheses -Wno-unused-function \
#    -Wno-unused -Wno-documentation -fdiagnostics-show-option \
#    -pthread ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

SET(CMAKE_CXX_FLAGS "-Wall -Wextra" ${CMAKE_CXX_FLAGS})

if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    
    #Using libc++ as standard library with clang
    SET(LIBCXX_INC_PATH ${PROJECT_ROOT}/llvm-build/output/include/c++/v1)
    SET(LIBCXX_LIB_PATH ${PROJECT_ROOT}/llvm-build/output/lib)
    # make sure to match the compiler flags with which the Unreal
    # Engine is built with
    set(CMAKE_CXX_FLAGS "\
        -nostdinc++ -ferror-limit=10 -isystem ${LIBCXX_INC_PATH} \
        -D__CLANG__ ${CMAKE_CXX_FLAGS}")

    # Need to link additional libraries with Clang to set up LLVM libc++
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
        -stdlib=libc++ -lc++ -lc++abi -lm -lc \
        -L ${LIBCXX_LIB_PATH} -rpath ${LIBCXX_LIB_PATH}")

    if("${CMAKE_BUILD_TYPE}" STREQUAL "debug")
        # set same options that Unreal sets in debug builds
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
            -funwind-tables -fdiagnostics-format=msvc \
            -fno-inline -fno-omit-frame-pointer -fstack-protector")
    endif()

else()
    set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept \
        -Wstrict-null-sentinel")
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    OpenCV
)

find_package(Threads REQUIRED)

message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
	${AirLib_dir}/deps/eigen3
    ${AirLib_dir}/deps/rpclib/include
	${AirLib_dir}/include
    ${fisheye_dir}/include
    ${RPCLIB_INCLUDES}
    ${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
)

link_directories(
    ${AirLib_dir}/lib
    ${AirLib_dir}/deps/rpclib/lib
    ${LLVM_DIR}/lib
)

add_executable(
    ${PROJECT_NAME}
    src/ros_api.cpp
    ${fisheye_dir}/src/fisheye.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    AirLib
    rpc
    pthread
    cv_bridge #This must be recompiled from source for curent version of ROS
)

add_dependencies(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
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
