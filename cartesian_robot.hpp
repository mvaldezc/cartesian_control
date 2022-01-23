#pragma once

#include "trajectory_inter.hpp"

namespace CartesianRobot{
    //double w = 0;
    //double pos_x = 0, pos_y = 0, pos_z = 0;
    Algorithm::TrajectoryGeneration::trajectory_segment_t *path_segment_buffer[3] = {nullptr, nullptr, nullptr};

    /**
     * @brief Read path specification and prepare for trajectory interpolation
     * @param[in] path Path specification
     */
    void set_trajectory_buffer(Algorithm::TrajectoryGeneration::path_t * path)
    {
        Algorithm::TrajectoryGeneration::path_t motor_path = *path;
        switch (path[0].path_type)
        {
            case Algorithm::TrajectoryGeneration::TrajectoryInterpolationType::linear_polynomial:
                path_segment_buffer[0] = new Algorithm::TrajectoryGeneration::linear_segment_t(motor_path.pos_i, motor_path.pos_f, motor_path.time_interval);
                break;
            case Algorithm::TrajectoryGeneration::TrajectoryInterpolationType::cubic_polynomial:
                path_segment_buffer[0] = new Algorithm::TrajectoryGeneration::cubic_segment_t(motor_path.pos_i, motor_path.pos_f, motor_path.time_interval);
                break;
            case Algorithm::TrajectoryGeneration::TrajectoryInterpolationType::quintic_polynomial:
                path_segment_buffer[0] = new Algorithm::TrajectoryGeneration::quintic_segment_t(motor_path.pos_i, motor_path.pos_f, motor_path.time_interval);
                break;
            case Algorithm::TrajectoryGeneration::TrajectoryInterpolationType::septic_polynomial:
                path_segment_buffer[0] = new Algorithm::TrajectoryGeneration::septic_segment_t(motor_path.pos_i, motor_path.pos_f, motor_path.time_interval);
                break;
        }
    }

    void clean_trajectory_buffer(){
        delete path_segment_buffer[0];
        delete path_segment_buffer[1];
        delete path_segment_buffer[2];
        path_segment_buffer[0] = nullptr;
        path_segment_buffer[1] = nullptr;
        path_segment_buffer[2] = nullptr;
    }
}