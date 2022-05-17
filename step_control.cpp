#include <stdio.h>
#include <string.h> 
#include <memory>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stepper.hpp"
#include "isr_sampling.hpp"
#include "trajectory_gen.hpp"
#include "cartesian_robot.hpp"

#define i2c_baud_rate 115200
#define i2c_slave_addr 0x02

//================ Global variables definition ================

using namespace Algorithm::TrajectoryGeneration;
using namespace Motor;

int main()
{
    stdio_init_all();
    printf("Uart init completed");

    #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #error i2c software component requires a board with I2C pins
    #endif
    // Use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, i2c_baud_rate);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    i2c_set_slave_mode(i2c0, true, i2c_slave_addr);
    uint8_t i2c_buffer = 0;
    while(true){
        i2c_read_raw_blocking(i2c0, &i2c_buffer, 1);
        i2c_buffer++;
        i2c_write_raw_blocking(i2c0, &i2c_buffer, 1);
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
