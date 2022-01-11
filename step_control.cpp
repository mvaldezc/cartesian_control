#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "stepper.hpp"
#include "isr_sampling.hpp"

//================== Constants definition ==================
constexpr uint64_t sampling_time_us = 200;
constexpr int max_blocking_cycles = 5;
constexpr double error_threshold_factor = 1.2;
constexpr uint64_t step_pulse_width_us = 50;

//================ Global variables definition ================
char motor_x_id[8] = "motor_x";
static Sample::TimerIsrSampler motor_sampler(sampling_time_us, max_blocking_cycles, error_threshold_factor);

bool move_motor_callback(struct repeating_timer *t)
{
    // If something is wrong with the timer, return false
    if (!motor_sampler.isr_time_check())
    {
        return false;
    }
    static_cast<Motor::Stepper *>(t->user_data)->step(step_pulse_width_us);
    //printf("String largo de prueba para romper el timer");
    return true;
}

int main()
{
    stdio_init_all();
    printf("System init completed");

    Motor::Stepper motor_x(motor_x_id, 200, 4, 5, 6);
    printf("Motor init completed");
    motor_x.enableMotor();
    printf("Motor enabled");

    motor_sampler.init(move_motor_callback);
    printf("Timer started");

    while (!motor_sampler.isr_data.timer_error_flag){}

    motor_sampler.cancel();
    printf("Timer cancelled");

    while(true);

    return 0;
}
