cmake_minimum_required(VERSION 3.2)
project(hen)

#set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
FIND_PACKAGE(X11 REQUIRED)
LINK_LIBRARIES ( ${X11_LIBRARIES} )

FIND_PACKAGE(Threads REQUIRED)
LINK_LIBRARIES ( ${CMAKE_THREAD_LIBS_INIT} )

include_directories(/usr/include/eigen3/)


set(SOURCE_FILES
    stdcomp/samplers.h
    stdcomp/shaders.h
    hen.h
    io.h
    main.cpp
    utils.h)

add_executable(hen ${SOURCE_FILES})