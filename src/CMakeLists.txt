#DART Tests
cmake_minimum_Required(VERSION 3.5.1)

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-gui dart-utils-urdf dart-gui-osg)
find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)

add_executable(urdf_load urdf_load.cpp)
target_link_libraries(urdf_load PUBLIC ${required_libraries})

add_executable(frame_test frame_test.cpp)
target_link_libraries(frame_test PUBLIC ${required_libraries})

add_executable(custom_node_test custom_node_test.cpp)
target_link_libraries(custom_node_test PUBLIC ${required_libraries})

