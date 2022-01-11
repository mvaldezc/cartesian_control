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
double a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0, w = 0;
int pos_inicial_x = 0, pos_final_x = 0, pos_x = 0, pos_anterior_x = 0;
int pos_inicial_y = 0, pos_final_y = 0, pos_y = 0, pos_anterior_y = 0;
int k = 0;

void set_third_poly_movement(int pos_inicial, int pos_final, int time_interval)
{
    // Time scaling w[t] ∈ [0,1]
    //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3
    // Boundary values:
    //    w[0] = 0, w[tf]=1, w'[0]=0, w'[tf]=0
    // Solving for constants:
    //   a0 = 0
    //   a1 = 0
    //   a2 = 3/tf^2
    //   a3 = -2/tf^3
    // Replacing in eq:
    //   w[t] = t^2 * (-2*t + 3*tf) / tf^3
    // In task space:
    //   P[w[t]] = (Pf - Po) * w +Po
    // Direct kinematics of cartesian robot with 2 DoF:
    //   Px = α * θx
    //   Py = β * θy
    // Since it is linear:
    //   θx[w[t]] = Px/α = (θxf - θxo) * w + θxo
    //   θy[w[t]] = Py/β = (θyf - θyo) * w + θyo

    a2 = 3 / (time_interval * time_interval);
    a3 = -2 / (time_interval * time_interval * time_interval);
    pos_inicial_x = pos_inicial;
    pos_final_x = pos_final;
}

float third_poly_move(int time){
    w = a2 * time * time + a3 * time * time * time;
    return w;
}

void move_motors_x_y(struct repeating_timer * t)
{
    k++;
    w = third_poly_move(k);
    pos_x = (pos_final_x - pos_inicial_x) * w + pos_inicial_x;
    //pos_y = (pos_final_y - pos_inicial_y) * w + pos_inicial_y;
    if (pos_x >= (pos_anterior_x+1)){
        static_cast<Motor::Stepper *>(t->user_data)->step(step_pulse_width_us);
        pos_anterior_x++;
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
    printf("System init completed");

    Motor::Stepper motor_x(motor_x_id, 200, 4, 5, 6);
    printf("Motor init completed");
    motor_x.enableMotor();
    printf("Motor enabled");
    set_third_poly_movement(0, 10, 50000);
    motor_sampler.init(move_motor_callback, &motor_x);
    printf("Timer started");

    while (!motor_sampler.isr_data.timer_error_flag){}

    motor_sampler.cancel();
    printf("Timer cancelled");

    while(true);

    return 0;
}
