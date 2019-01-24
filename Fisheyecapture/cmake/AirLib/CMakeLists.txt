cmake_minimum_required(VERSION 3.5.0)
project(AirLib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake_modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake_modules/CommonSetup.cmake")

CommonSetup()

include_directories(
  ${FISHEYECAPTURE_ROOT}/AirLib/deps/eigen3
  ${FISHEYECAPTURE_ROOT}/AirLib/
  ${FISHEYECAPTURE_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
)

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})