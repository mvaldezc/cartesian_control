# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(step_control C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(step_control step_control.cpp src/stepper.cpp trajectory_gen.cpp)

pico_set_program_name(step_control "step_control")
pico_set_program_version(step_control "0.1")

pico_enable_stdio_uart(step_control 1)

# Add include path
target_include_directories(step_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# Add any user requested libraries
target_link_libraries(step_control
        pico_stdlib
        pico_sync
        pico_multicore
        hardware_i2c
)

pico_add_extra_outputs(step_control)

