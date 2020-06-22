cmake_minimum_required (VERSION 3.10)
project(Dart_Tests)

#set(SOURCES ${PROJECT_SOURCE_DIR}/src/QuickStart.cpp)

#include_directories("${PROJECT_SOURCE_DIR}/include")
#add_library(QuickStart STATIC ${SOURCES})
#enable_testing()

add_subdirectory(src)
add_subdirectory(src/nomad_controller)