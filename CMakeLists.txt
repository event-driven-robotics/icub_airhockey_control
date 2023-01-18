cmake_minimum_required(VERSION 3.0)
project(icub_airhockey_control)
find_package(BipedalLocomotionFramework REQUIRED)
find_package(YARP REQUIRED)
add_executable(icub_airhockey_control main.cpp)
target_link_libraries(icub_airhockey_control PRIVATE BipedalLocomotion::IK 
                                                     BipedalLocomotion::ContinuousDynamicalSystem 
                                                     BipedalLocomotion::ParametersHandlerYarpImplementation
                                                     YARP_dev)
