#DART Tests
cmake_minimum_Required(VERSION 3.5.1)

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-gui dart-utils-urdf dart-gui-osg)
find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)

add_executable(urdf_load urdf_load.cpp)
target_link_libraries(urdf_load PUBLIC ${required_libraries})

