#DART Tests
cmake_minimum_Required(VERSION 3.5.1)

set(NOMAD_CONTROLLER_SOURCES ${PROJECT_SOURCE_DIR}/src/nomad_controller/src/FiniteStateMachine.cpp
${PROJECT_SOURCE_DIR}/src/nomad_controller/src/State.cpp
${PROJECT_SOURCE_DIR}/src/nomad_controller/src/TransitionEvent.cpp
${PROJECT_SOURCE_DIR}/src/nomad_controller/src/nomad_controller.cpp

#${PROJECT_SOURCE_DIR}/src/nomad_controller/src/CrouchState.cpp
#${PROJECT_SOURCE_DIR}/src/nomad_controller/src/StandState.cpp
#${PROJECT_SOURCE_DIR}/src/nomad_controller/src/IdleState.cpp
#${PROJECT_SOURCE_DIR}/src/nomad_controller/src/EmergencyStopState.cpp
${PROJECT_SOURCE_DIR}/src/nomad_controller/src/CubicPolynomialTrajectory.cpp
)


set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-gui dart-utils-urdf dart-gui-osg)
find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)


include_directories("${PROJECT_SOURCE_DIR}/src/nomad_controller/include")

add_executable(nomad_controller ${NOMAD_CONTROLLER_SOURCES})
target_link_libraries(nomad_controller PUBLIC ${required_libraries})

