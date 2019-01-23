cmake_minimum_required(VERSION 3.2)
project(hen)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options("-march=native" "-Wdouble-promotion" "-fno-math-errno")

set(SOURCE_FILES
    stdcomp/samplers.h
    stdcomp/shaders.h
    hen.h
    io.h
    main.cpp
    utils.h veclib.h)

add_executable(hen ${SOURCE_FILES})

add_subdirectory(cfw)
target_link_libraries(hen PRIVATE cfw_lib)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
target_link_libraries(hen PRIVATE ${PNG_LIBRARY})