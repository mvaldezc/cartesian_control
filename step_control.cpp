#include <cstdio>
#include <string.h> 
#include <memory>
#include "pico/stdlib.h"
#include "stepper.hpp"
#include "cartesian_robot.hpp"
#include "i2c_slave.hpp"
#include "data_reader.hpp"
#include "state_manager.hpp"

//================ Global variables definition ================

using namespace Algorithm::TrajectoryGeneration;
using namespace Motor;
using namespace Communication;
using namespace System::Robot;

int main()
{
    stdio_init_all();
    printf("Uart init completed\n");

    StateManager * stateManager = StateManager::getInstance();

    I2CSlave::init(&rxCallback, &txCallback);
    printf("I2C init completed\n");

    while(true){   
        busy_wait_ms(40);
    }

    /*
    std::shared_ptr<IMotor> motor_x(new Stepper(200, 4, 5, 6));
    std::shared_ptr<IMotor> motor_y(new Stepper(200, 7, 8, 9));
    std::shared_ptr<IMotor> motor_z(new Stepper(200, 10, 11, 12));

    CartesianRobotClient welding_system(motor_x, motor_y, motor_z);

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

    printf("Starting trajectory");
    welding_system.execute_routine(5, via_points);
    printf("Motor disabled");

    

    while (true);
    */
    return 0;
}
