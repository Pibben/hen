cmake_minimum_required(VERSION 3.2)
project(hen)

set(CMAKE_CXX_COMPILER clang++-3.7)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -stdlib=libc++ -Wno-deprecated-register -march=native")
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