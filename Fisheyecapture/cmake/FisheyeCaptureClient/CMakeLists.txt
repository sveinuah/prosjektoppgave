cmake_minimum_required(VERSION 3.5.0)
project(FisheyeCaptureClient)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake_modules")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake_modules/CommonSetup.cmake")

CommonSetup()
SetupConsoleBuild()

include_directories(
	${FISHEYECAPTURE_ROOT}/AirLib/deps/eigen3
	${FISHEYECAPTURE_ROOT}/FisheyeCaptureClient/src
	${FISHEYECAPTURE_ROOT}/AirLib/include
	${RPC_LIB_INCLUDES}
)

AddExecutableSource()

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})

