/***********************************************************************
 * @file	:	cartesian_robot.hpp
 * @brief 	:	3 DoF Cartesian Robot Library
 * 				Library to move a cartesian robot through via points.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once

#include <memory>
#include <utility>
#include "trajectory_gen.hpp"
#include "trajectory_data.hpp"
#include "stepper.hpp"
#include "isr_sampling.hpp"

using namespace Algorithm::TrajectoryGeneration;
using namespace Motor;
using namespace Sampler;

namespace System::Robot {

    typedef struct
    {
        float maxAbsDist_cm;
        float minAbsDist_cm;
        uint motorResolution_um;
        uint rotationToLinearFactor_rev_cm;
        uint maxSpeed_cm_s;
    } AxisSettings_t;

    //================== Constants definition ==================
    constexpr uint64_t sampling_period_us = 250; // 200
    constexpr uint64_t step_pulse_width_us = 40; // 50
    constexpr double sampling_period_sec = (double)sampling_period_us / 1000000;
    constexpr float um_to_cm_factor = (float) 1/10000;

    //================ Global variables definition ================
    static TimerIsrSampler motor_sampler(sampling_period_us);

    /**
     * @class CartesianRobotClient
     * @brief Manages the resources required for a classic cartesian robot.
     * @details
     * -Hardware: It allows controlling up to 3 motors and 1 timer.
     * -Algorithms: It allows interpolation algorithms for trajectory generation.
     * -Data: It stores data structure with via points.
     */
    class CartesianRobotClient
    {
        public:
            CartesianRobotClient(std::shared_ptr<IMotor> motor_x, std::shared_ptr<IMotor> motor_y, 
                std::shared_ptr<IMotor> motor_z)
                : motor_x(std::move(motor_x)), motor_y(std::move(motor_y)), motor_z(std::move(motor_z))
            {
                // TODO check what happens if motors are nullptr
                move_motor_timer_callback =
                    [](void * t)
                    {
                        static_cast<CartesianRobotClient * >(t)->move_motors();
                        return true;
                    };
            }

            ITrajectoryInterpolation * path_segment_buffer[3] = {nullptr, nullptr, nullptr};
            path_params_t * path_list;
            path_params_t * next_path;
            std::shared_ptr<IMotor> motor_x;
            std::shared_ptr<IMotor> motor_y;
            std::shared_ptr<IMotor> motor_z;


            /**
             * @brief send enable signal to motors.
             */
            void enable_motors();

            /**
             * @brief send disable signal to motors.
             */
            void disable_motors();

            /**
             * @brief Execute a series of movements using 3 motors.
             * @param[in] list_size Quantity of path segments.
             * @param[in] path_list_ptr Pointer to path segments list.
             */
            void execute_routine(size_t list_size, path_params_t * path_list_ptr);

            /**
             * @brief Check if motors are ready to move and move them.
             */
            void move_motors();

            AxisSettings_t x_axis;
            AxisSettings_t y_axis;
            AxisSettings_t z_axis;

        private:

            volatile double w = 0;
            volatile int k = 0;
            volatile double cnt;
            volatile bool finished = false;
            volatile int pos_x = 0, pos_anterior_x = 0;
            volatile int pos_y = 0, pos_anterior_y = 0;
            volatile int pos_z = 0, pos_anterior_z = 0;

            isrTimerCallback_t move_motor_timer_callback = nullptr;

            /**
             * @brief Save list of path segments.
             */
            void save_path_list(path_params_t *path_list_ptr);

            /**
             * @brief Prepare for next path segment interpolation.
             */
            void set_trajectory_buffer();

            /**
             * @brief Set rotation direction of the motors.
             */
            void set_next_motor_direction();

            /** 
             * @brief Clean next path segment pointer.
             */
            void clean_trajectory_buffer();

            /**
             * @brief Avoid motor for passing an established absolute limit range.
             * @return True if limits are reached.
             */
            bool openLoopLimitCheck(IMotor * motor);

    };

} // namespace System::Robot
