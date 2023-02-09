cmake_minimum_required(VERSION 3.0)
project(icub_airhockey_control)
find_package(BipedalLocomotionFramework REQUIRED)
find_package(YARP REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_library(common include/icubAirHockeyControlModule.h
        include/controlBoardWrapper.h 
        src/controlBoardWrapper.cpp
        src/icubAirHockeyControlModule.cpp)

target_link_libraries(common BipedalLocomotion::IK 
                      BipedalLocomotion::ContinuousDynamicalSystem 
                      BipedalLocomotion::ParametersHandlerYarpImplementation)

add_executable(icub_airhockey_control main.cpp)
target_link_libraries(icub_airhockey_control PRIVATE common)
