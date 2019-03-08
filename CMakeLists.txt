cmake_minimum_required(VERSION 3.2)
project(hen)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options("-march=native" "-Wdouble-promotion" "-fno-math-errno")

add_library(hen_lib INTERFACE)

target_include_directories(hen_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
target_link_libraries(hen_lib INTERFACE ${PNG_LIBRARY})

add_subdirectory(cfw)
target_link_libraries(hen_lib INTERFACE cfw_lib)

add_executable(cow examples/cow.cpp)
set_target_properties(cow PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")
target_link_libraries(cow PRIVATE hen_lib)

add_executable(test examples/test.cpp)
set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")
target_link_libraries(test PRIVATE hen_lib)

add_executable(shadertoy examples/shadertoy.cpp)
set_target_properties(shadertoy PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")
target_link_libraries(shadertoy PRIVATE hen_lib)

add_executable(imgui examples/imgui/imgui_impl.cpp examples/imgui/imgui/imgui.cpp examples/imgui/imgui/imgui_draw.cpp examples/imgui/imgui/imgui_widgets.cpp examples/imgui/imgui/imgui_demo.cpp)
set_target_properties(imgui PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")
target_link_libraries(imgui PRIVATE hen_lib)
