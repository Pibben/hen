cmake_minimum_required(VERSION 3.2)
project(hen)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options("-march=native" "-Wdouble-promotion")
FIND_PACKAGE(X11 REQUIRED)
LINK_LIBRARIES ( ${X11_LIBRARIES} )

FIND_PACKAGE(Threads REQUIRED)
LINK_LIBRARIES ( ${CMAKE_THREAD_LIBS_INIT} )

set(SOURCE_FILES
    stdcomp/samplers.h
    stdcomp/shaders.h
    hen.h
    io.h
    main.cpp
    utils.h veclib.h)

add_executable(hen ${SOURCE_FILES})