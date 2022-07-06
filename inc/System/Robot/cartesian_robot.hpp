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
#include <unordered_map>
#include <functional>
#include "atomic.hpp"
#include "trajectory_gen.hpp"
#include "trajectory_data.hpp"
#include "stepper.hpp"
#include "isr_sampling.hpp"

using namespace Algorithm::TrajectoryGeneration;
using namespace Motor;
using namespace Sampler;

namespace System::Robot {

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
            CartesianRobotClient(
                std::shared_ptr<IMotor> motor_x, std::shared_ptr<IMotor> motor_y, 
                std::shared_ptr<IMotor> motor_z, TimerIsrSampler & motor_sampler)
                : motor_x(std::move(motor_x)), motor_y(std::move(motor_y)), 
                motor_z(std::move(motor_z)), motor_sampler(motor_sampler),
                sampling_period_sec(motor_sampler.getSamplingPeriod_us()/1000000.0)
            {
                move_motor_timer_callback =
                    [](void * t)
                    {
                        static_cast<CartesianRobotClient * >(t)->move_motors();
                        return true;
                    };
                path_segment_buffer.insert({"x", nullptr});
                path_segment_buffer.insert({"y", nullptr});
                path_segment_buffer.insert({"z", nullptr});
            }


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
             * @param[in] path_queue_ptr Pointer to path segments queue.
             * @param[in] stopCallback Callable that should return true if the robot is required to stop.
             */
            void execute_routine(path_list_t path_queue_ptr, std::function<bool()> stopCallback);

            void resume_execution();


        private:
            std::unordered_map<std::string, std::unique_ptr<ITrajectoryInterpolation> > path_segment_buffer;
            
            path_list_t path_list;
            path_params_t next_path;
            
            std::shared_ptr<IMotor> motor_x;
            std::shared_ptr<IMotor> motor_y;
            std::shared_ptr<IMotor> motor_z;

            TimerIsrSampler & motor_sampler;
            const double sampling_period_sec;
            isrTimerCallback_t move_motor_timer_callback = nullptr;

            std::function<bool()> stopCallbackFnc;
            bool pause_flag;
            std::list<path_params_t>::iterator last_iter;
            volatile bool finished = false;

            volatile double w = 0;
            volatile int k = 0;
            volatile double cnt = 0;
            
            volatile int pos_x = 0, pos_anterior_x = 0;
            volatile int pos_y = 0, pos_anterior_y = 0;
            volatile int pos_z = 0, pos_anterior_z = 0;

            
            /**
             * @brief Check if motors are ready to move and move them.
             */
            void move_motors();

            /**
             * @brief Save list of path segments.
             */
            inline void save_path_list(path_list_t & path_list_ptr)
            {
                path_list = std::move(path_list_ptr);
            }

            /**
             * @brief Prepare for next path segment interpolation.
             */
            void set_trajectory_buffer();


            /** 
             * @brief Clean next path segment pointer.
             */
            void clean_trajectory_buffer();

            void clear_vars();

            void mainExecution();
    };

} // namespace System::Robot
