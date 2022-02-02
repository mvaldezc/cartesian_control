#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "stepper.hpp"
#include "isr_sampling.hpp"
#include "trajectory_gen.hpp"
#include "cartesian_robot.hpp"

//================== Constants definition ==================
constexpr uint64_t sampling_period_us = 200; //200
constexpr int max_blocking_cycles = 5;
constexpr double error_threshold_factor = 1.2;
constexpr uint64_t step_pulse_width_us = 50; //50

//================ Global variables definition ================
char motor_x_id[8] = "motor_x";
static Sampler::TimerIsrSampler motor_sampler(sampling_period_us, max_blocking_cycles, error_threshold_factor);
double w = 0;
int pos_inicial_x = 0, pos_final_x = 0, pos_x = 0, pos_anterior_x = 0;
int pos_inicial_y = 0, pos_final_y = 0, pos_y = 0, pos_anterior_y = 0;
int k = 0;
bool finished = false;

using namespace Algorithm::TrajectoryGeneration;

void move_motors_x_y(struct repeating_timer * t)
{
    k++;
    w = CartesianRobot::path_segment_buffer[0]->poly_move((double)k/5000.0);//5000.0
    pos_x = (CartesianRobot::path_segment_buffer[0]->pos_f - CartesianRobot::path_segment_buffer[0]->pos_i) * w + CartesianRobot::path_segment_buffer[0]->pos_i;
    //pos_y = (pos_final_y - pos_inicial_y) * w + pos_inicial_y;
    if (pos_x == (pos_anterior_x+1)){
        static_cast<Motor::Stepper *>(t->user_data)->step(step_pulse_width_us);
        pos_anterior_x++;
    }
    if (pos_x == CartesianRobot::path_segment_buffer[0]->pos_f)
    {
        finished = true;
    }
}

bool move_motor_callback(struct repeating_timer * t)
{
    // If something is wrong with the timer, return false
    if (!motor_sampler.isr_time_check())
    {
        return false;
    }
    move_motors_x_y(t);
    // printf("String largo de prueba para romper el timer");
    return true;
}

int main()
{
    stdio_init_all();
    printf("Uart init completed");

    Motor::Stepper motor_x(motor_x_id, 200, 4, 5, 6);
    printf("Motor init completed");

    motor_x.setDirection(Motor::MotorDirection::CounterClockwise);
    motor_x.enableMotor();
    printf("Motor enabled");

    path_t via_points[4] = {
        {TrajectoryInterpolationType::linear_polynomial, 0, 1000, 5},
        {TrajectoryInterpolationType::cubic_polynomial, 0, 1000, 5},
        {TrajectoryInterpolationType::quintic_polynomial, 0, 1000, 5},
        {TrajectoryInterpolationType::septic_polynomial, 0, 1000, 5}
    }; // max 8000, 5

    printf("Starting trajectory");
    for (int i = 0; i < 4; i++){
        CartesianRobot::set_trajectory_buffer(&via_points[i]);
        motor_sampler.init(move_motor_callback, &motor_x);
        while (!motor_sampler.timer_error_flag && !finished){}
        motor_sampler.cancel();
        CartesianRobot::clean_trajectory_buffer();
        k = 0;
        pos_anterior_x = 0;
        finished = false;
    }

    motor_x.disableMotor();
    printf("Motor disabled");

    while(true);

    return 0;
}
