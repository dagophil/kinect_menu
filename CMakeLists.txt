cmake_minimum_required(VERSION 2.8)

project(kinect_menu)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/config)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Fix for Qt Creator to list the hxx files.
file(GLOB HEADERS "include/*.hxx")
add_custom_target(headerfiles SOURCES ${HEADERS})

# Choose Release build by default
if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif()

# C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# SFML
find_package(SFML REQUIRED system window graphics audio)
include_directories(${SFML_INCLUDE_DIR})

# OpenNI
find_package(OpenNI REQUIRED)
include_directories(${OPENNI_INCLUDE_DIRS})

# The executables
add_executable(kinect_menu kinect_menu.cxx)
target_link_libraries(kinect_menu
    ${SFML_LIBRARIES}
    ${OPENNI_LIBRARIES}
)
