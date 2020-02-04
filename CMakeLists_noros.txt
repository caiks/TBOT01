cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(TBOT01)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (Threads)

add_subdirectory(../AlignmentRepaC ../AlignmentRepaC_build)

include_directories(../rapidjson/include ../AlignmentC ../AlignmentRepaC)

IF( "${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC" )
include_directories(C:/dev/ros2-windows/include)
ENDIF ()

add_executable(controller controller.cpp)

add_executable(TBOT01 main.cpp dev.cpp )

target_link_libraries(controller PUBLIC AlignmentRepaC AlignmentC Threads::Threads)

target_include_directories(controller PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          )

target_link_libraries(TBOT01 PUBLIC AlignmentRepaC AlignmentC Threads::Threads)

target_include_directories(TBOT01 PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          )

