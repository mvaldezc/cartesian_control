#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "stepper.hpp"
#include "isr_sampling.hpp"
#include "trajectory_gen.hpp"
#include "cartesian_robot.hpp"

//================ Global variables definition ================
char motor_x_id[8] = "motor_x", motor_y_id[8] = "motor_y", motor_z_id[8] = "motor_z";

using namespace Algorithm::TrajectoryGeneration;
using namespace Motor;


int main()
{
    stdio_init_all();
    printf("Uart init completed");
    IMotor * motor_x = new Stepper(motor_x_id, 200, 4, 5, 6);
    IMotor * motor_y = new Stepper(motor_y_id, 200, 7, 8, 9);
    IMotor * motor_z = new Stepper(motor_z_id, 200, 10, 11, 12);

    CartesianRobotClient robot(motor_x, motor_y, motor_z);

    path_params_t via_points[5] = {
        {.path_type = static_cast<char>(InterpolationType::smooth_polynomial),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 4 * 4000,
         .pos_x = 4000,
         .pos_y = 4000,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::trapezoid_polynomial),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 2 * 4000,
         .pos_x = 500,
         .pos_y = 0,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::trapezoid_polynomial),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 2 * 4000,
         .pos_x = 0,
         .pos_y = 500,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::smooth_polynomial),
         .dir_x = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 3 * 4000,
         .pos_x = 4000,
         .pos_y = 4000,
         .pos_z = 0},
        {.path_type = static_cast<char>(InterpolationType::smooth_polynomial),
         .dir_x = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 3 * 4000,
         .pos_x = 2000,
         .pos_y = 4000,
         .pos_z = 0}};

    printf("Starting trajectory");
    robot.execute_routine(5, via_points);
    printf("Motor disabled");

    while(true);

    return 0;
}
