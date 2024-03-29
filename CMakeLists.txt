cmake_minimum_required(VERSION 3.16.3)
project(dflog2ubx LANGUAGES CXX)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

if(NOT DEFINED ARDUPILOT_FOLDER)
    message(FATAL_ERROR "ARDUPILOT_FOLDER is not set")
endif()

add_executable(
	dflog2ubx
	dflog2ubx.cpp
)

include_directories(
	${ARDUPILOT_FOLDER}/libraries
	${ARDUPILOT_FOLDER}/build/sitl/libraries/GCS_MAVLink
)