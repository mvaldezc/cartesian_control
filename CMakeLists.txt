# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/rasp/Documentos/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(step_control C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(step_control 
        step_control.cpp 
        state_manager.hpp
        src/Motor/stepper.cpp 
        src/Algorithm/TrajectoryGeneration/trajectory_gen.cpp
        src/System/Robot/cartesian_robot.cpp
        src/Communication/i2c_slave.cpp
        src/Machine/state_manager.cpp
)

pico_set_program_name(step_control "step_control")
pico_set_program_version(step_control "0.1")

pico_enable_stdio_uart(step_control 1)

# Add include path
target_include_directories(step_control 
        PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/Algorithm/TrajectoryGeneration
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/Motor
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/Sampler
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/Communication
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/System/Robot
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/Machine
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/Mutex
        ${CMAKE_CURRENT_SOURCE_DIR}/
)

# Select platform (#define platform macro)
target_compile_definitions(step_control PUBLIC RASP_PICO)

# Add any user requested libraries
target_link_libraries(step_control
        pico_stdlib
        pico_sync
        pico_multicore
        hardware_i2c
)

pico_add_extra_outputs(step_control)

