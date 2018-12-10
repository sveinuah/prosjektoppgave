cmake_minimum_required(VERSION 3.5.0)
project(FisheyeCaptureClient)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake_modules")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake_modules/CommonSetup.cmake")

CommonSetup()
SetupConsoleBuild()

find_package( OpenCV  3 REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${FISHEYECAPTURE_ROOT}/AirLib/deps/eigen3
	${FISHEYECAPTURE_ROOT}/FisheyeCaptureClient/include
	${FISHEYECAPTURE_ROOT}/AirLib/include
	${RPC_LIB_INCLUDES}
)

AddExecutableSource()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})

