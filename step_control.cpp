#include <cstdio>
#include <memory>
#include "pico/stdlib.h"
#include "i2c_slave.hpp"
#include "communication_handler.hpp"
#include "cartesian_robot.hpp"
#include "state_manager.hpp"

//================ Global variables definition ================

using namespace Algorithm::TrajectoryGeneration;
using namespace Communication;
using namespace Motor;
using namespace Sampler;
using namespace System::Robot;

int main()
{
    //================= Serial initialization ==================
    stdio_init_all();
    printf("Uart init completed\n");

    //============== State Manager initialization ==============
    StateManager * stateManager = StateManager::getInstance();
    printf("State Machine init completed\n");

    //============== Data Structure instantiation ==============
    auto via_points = std::make_shared<std::queue<path_params_t, std::list<path_params_t>>>();
    printf("Data Structure instantiated\n");

    //==================== Motor definition ====================
    std::shared_ptr<IMotor> motor_x(new Stepper(200, 4, 5, 6));
    std::shared_ptr<IMotor> motor_y(new Stepper(200, 7, 8, 9));
    std::shared_ptr<IMotor> motor_z(new Stepper(200, 10, 11, 12));
    printf("Motor def completed\n");

    //==================== Timer definition ====================
    constexpr uint64_t sampling_period_us = 250; // 200
    TimerIsrSampler motor_sampler(sampling_period_us);
    printf("Timer def completed\n");

    //============= Cartesian Robot initialization =============
    CartesianRobotClient welding_system(motor_x, motor_y, motor_z, motor_sampler);
    printf("Robot init completed\n");

    //=============== Data Handler initialization ===============
    installDataContainer(via_points);
    printf("Data Handler init completed\n");

    //=================== I2C initialization ===================
    I2CSlave::init(&rxCallback, &txCallback);
    printf("I2C init completed\n");

    printf("System Fully Initialized\n");

    via_points->push(
        {.path_type = static_cast<char>(InterpolationType::SmoothPoly),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 4 * 4000,
         .pos_x = 4000,
         .pos_y = 4000,
         .pos_z = 0});

    welding_system.execute_routine(via_points);

    while(true){   
        busy_wait_ms(40);
    }
    return 0;

    /*

    path_params_t via_points[5] = {
        {.path_type = static_cast<char>(InterpolationType::SmoothPoly),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 4 * 4000,
         .pos_x = 4000,
         .pos_y = 4000,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::TrapezoidPoly),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 2 * 4000,
         .pos_x = 500,
         .pos_y = 0,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::TrapezoidPoly),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 2 * 4000,
         .pos_x = 0,
         .pos_y = 500,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::SmoothPoly),
         .dir_x = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 3 * 4000,
         .pos_x = 4000,
         .pos_y = 4000,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::SmoothPoly),
         .dir_x = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 3 * 4000,
         .pos_x = 2000,
         .pos_y = 4000,
         .pos_z = 0}};
    */
}
