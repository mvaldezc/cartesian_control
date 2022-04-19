#pragma once

#include <memory>
#include "trajectory_gen.hpp"
#include "stepper.hpp"
#include "isr_sampling.hpp"

using namespace Algorithm::TrajectoryGeneration;
using namespace Motor;
using namespace Sampler;

//================== Constants definition ==================
constexpr uint64_t sampling_period_us = 250; // 200
constexpr uint64_t step_pulse_width_us = 40; // 50
constexpr double sampling_period_sec = (double)sampling_period_us / 1000000;

//================ Global variables definition ================
TimerIsrSampler motor_sampler(sampling_period_us);

double w = 0;
int k = 0;
double cnt;
bool finished = false;
int pos_x = 0, pos_anterior_x = 0;
int pos_y = 0, pos_anterior_y = 0;
int pos_z = 0, pos_anterior_z = 0;

bool move_motor_callback(struct repeating_timer *t);

class CartesianRobotClient
{
    public:
        CartesianRobotClient(std::shared_ptr<IMotor> motor_x, std::shared_ptr<IMotor> motor_y, std::shared_ptr<IMotor> motor_z)
            : motor_x(motor_x), motor_y(motor_y), motor_z(motor_z) {}

        ITrajectoryInterpolation * path_segment_buffer[3] = {nullptr, nullptr, nullptr};
        path_params_t * path_list;
        path_params_t * next_path;
        std::shared_ptr<IMotor> motor_x;
        std::shared_ptr<IMotor> motor_y;
        std::shared_ptr<IMotor> motor_z;

        /**
         * @brief Read list of path segments
         */
        void get_path_list(path_params_t *path_list_ptr)
        {
            path_list = path_list_ptr;
            next_path = path_list;
        }

        /**
         * @brief Read path list and prepare for next path segment interpolation.
         */
        void set_trajectory_buffer() 
        {
            set_buffer(motor_x.get(), path_segment_buffer[0], static_cast<InterpolationType>(next_path->path_type), next_path->pos_x, static_cast<MotorDirection>(next_path->dir_x), (next_path->time) / 4000.0);
            set_buffer(motor_y.get(), path_segment_buffer[1], static_cast<InterpolationType>(next_path->path_type), next_path->pos_y, static_cast<MotorDirection>(next_path->dir_y), (next_path->time) / 4000.0);
            set_buffer(motor_z.get(), path_segment_buffer[2], static_cast<InterpolationType>(next_path->path_type), next_path->pos_z, static_cast<MotorDirection>(next_path->dir_z), (next_path->time) / 4000.0);
        }

        /**
         * @brief Prepare trajectory interpolation for a specific axis and prepare motor direction.
         * @param[in] motor Motor pointer.
         * @param[out] path_segment_ptr Pointer to trajectory interpolation that will be used in next path.
         * @param[in] path_type Type of the next required interpolation.
         * @param[in] delta_pos Amount of steps for next path_segment.
         * @param[in] delta_time Amount of time for next_path_segment.
         */
        void set_buffer(IMotor * motor, ITrajectoryInterpolation * & path_segment_ptr, InterpolationType path_type, 
                        unsigned int delta_pos, MotorDirection dir, double delta_time)
        {
            // Allocate interpolation class in heap depending on interpolation type.
            switch (path_type) {
                case InterpolationType::linear_polynomial:
                    path_segment_ptr = new LinearInterpolation(delta_pos, delta_time);
                    break;
                case InterpolationType::cubic_polynomial:
                    path_segment_ptr = new CubicInterpolation(delta_pos, delta_time);
                    break;
                case InterpolationType::quintic_polynomial:
                    path_segment_ptr = new QuinticInterpolation(delta_pos, delta_time);
                    break;
                case InterpolationType::septic_polynomial:
                    path_segment_ptr = new SepticInterpolation(delta_pos, delta_time);
                    break;
                case InterpolationType::trapezoid_polynomial:
                    path_segment_ptr = new TrapezoidInterpolation(delta_pos, delta_time);
                    break;
                case InterpolationType::smooth_polynomial:
                    path_segment_ptr = new SmoothInterpolation(delta_pos, delta_time);
                    break;
            }
            // Prepare motor direction.
            motor->setDirection(dir);
        }

        /**
         * @brief send enable signal to motors.
         */
        void enable_motors() {
            motor_x->enableMotor();
            motor_y->enableMotor();
            motor_z->enableMotor();
        }

        /**
         * @brief send disable signal to motors.
         */
        void disable_motors() {
            motor_x->disableMotor();
            motor_y->disableMotor();
            motor_z->disableMotor();
        }

        /**
         * @brief Execute a series of movements using 3 motors.
         * @param[in] list_size Quantity of path segments.
         * @param[in] path_list_ptr Pointer to path segments list.
         */
        void execute_routine(size_t list_size, path_params_t * path_list_ptr)
        {
            enable_motors();
            get_path_list(path_list_ptr);
            for (int i = 0; i < list_size; i++)
            {
                set_trajectory_buffer();
                motor_sampler.init(move_motor_callback, this);
                while (!motor_sampler.timer_error_flag && !finished);
                motor_sampler.cancel();
                clean_trajectory_buffer(); 
                k = 0;
                pos_anterior_x = 0;
                pos_x = 0;
                pos_anterior_y = 0;
                pos_y = 0;
                pos_anterior_z = 0;
                pos_z = 0;
                finished = false;
            }
            disable_motors();
        }

        /** 
         * @brief Clean next path segment pointer.
         */
        void clean_trajectory_buffer(){
            delete path_segment_buffer[0];
            delete path_segment_buffer[1];
            delete path_segment_buffer[2];
            path_segment_buffer[0] = nullptr;
            path_segment_buffer[1] = nullptr;
            path_segment_buffer[2] = nullptr;
            next_path++;
        }

        void move_motors_x_y()
        {
            k++;
            cnt = (double)k * sampling_period_sec;
            w = this->path_segment_buffer[0]->interpolateMotion(cnt);
            pos_x = (this->path_segment_buffer[0]->d_pos) * w;
            // pos_y = (pos_final_y - pos_inicial_y) * w + pos_inicial_y;
            if (pos_x == (pos_anterior_x + 1) || cnt >= this->path_segment_buffer[0]->d_time)
            {
                static_cast<Motor::Stepper *>(motor_x.get())->step(step_pulse_width_us);
                pos_anterior_x++;
                if (pos_anterior_x == this->path_segment_buffer[0]->d_pos)
                {
                    finished = true;
                }
            }

            w = this->path_segment_buffer[1]->interpolateMotion(cnt);
            pos_y = (this->path_segment_buffer[1]->d_pos) * w;
            if (pos_y == (pos_anterior_y + 1) || cnt >= this->path_segment_buffer[1]->d_time)
            {
                static_cast<Motor::Stepper *>(motor_y.get())->step(step_pulse_width_us);
                pos_anterior_y++;
                if (pos_anterior_y == this->path_segment_buffer[1]->d_pos)
                {
                    finished = true;
                }
            }

            w = this->path_segment_buffer[2]->interpolateMotion(cnt);
            pos_z = (this->path_segment_buffer[2]->d_pos) * w;
            if (pos_z == (pos_anterior_z + 1) || cnt >= this->path_segment_buffer[2]->d_time)
            {
                static_cast<Motor::Stepper *>(motor_z.get())->step(step_pulse_width_us);
                pos_anterior_z++;
                if (pos_anterior_z == this->path_segment_buffer[2]->d_pos)
                {
                    finished = true;
                }
            }
        }

        
};

bool move_motor_callback(struct repeating_timer *t)
{
    // If something is wrong with the timer, return false
    if (!motor_sampler.isr_time_check())
    {
        return false;
    }
    static_cast<CartesianRobotClient *>(t->user_data)->move_motors_x_y();
    // printf("String largo de prueba para romper el timer");
    return true;
}
