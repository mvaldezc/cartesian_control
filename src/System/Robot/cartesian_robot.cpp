#include "cartesian_robot.hpp"

namespace System::Robot {

    void CartesianRobotClient::set_trajectory_buffer() 
    {
        if(motor_x)
        {
            motor_x->setDirection(static_cast<MotorDirection>(next_path.dir_x));
            path_segment_buffer["x"] = InterpolationFactory::create(static_cast<InterpolationType>(next_path.path_type),
            next_path.pos_x, (next_path.time) / 4000.0);
        }
        if(motor_y)
        {
            motor_y->setDirection(static_cast<MotorDirection>(next_path.dir_y));
            path_segment_buffer["y"] = InterpolationFactory::create(static_cast<InterpolationType>(next_path.path_type),
            next_path.pos_y, (next_path.time) / 4000.0);
        }
        if(motor_z)
        {
            motor_z->setDirection(static_cast<MotorDirection>(next_path.dir_z));
            path_segment_buffer["z"] = InterpolationFactory::create(static_cast<InterpolationType>(next_path.path_type),
            next_path.pos_z, (next_path.time) / 4000.0);
        }
    }

    void CartesianRobotClient::enable_motors() {
        if(motor_x)
            motor_x->enableMotor();
        if(motor_y)
            motor_y->enableMotor();
        if(motor_z)
            motor_z->enableMotor();
    }

    void CartesianRobotClient::disable_motors() {
        if(motor_x)
            motor_x->disableMotor();
        if(motor_y)
            motor_y->disableMotor();
        if(motor_z)
            motor_z->disableMotor();
    }

    void CartesianRobotClient::execute_routine(path_list_t path_list_ptr)
    {
        save_path_list(path_list_ptr);
        if(path_list)
        {
            enable_motors();
            for(auto it = path_list->begin(); it != path_list->end(); ++it)
            {
                next_path = *it;
                set_trajectory_buffer();
                motor_sampler.init(move_motor_timer_callback, this);
                while (!motor_sampler.errorFlag && !finished);
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
    }

    void CartesianRobotClient::clean_trajectory_buffer(){
        path_segment_buffer["x"].reset(nullptr);
        path_segment_buffer["y"].reset(nullptr);
        path_segment_buffer["z"].reset(nullptr);
    }

    void CartesianRobotClient::move_motors()
    {
        cnt = (++k) * sampling_period_sec;
        
        // pos = (pos_f - pos_i) * w + pos_i;
        if(path_segment_buffer["x"] != nullptr)
        {
            w = path_segment_buffer["x"]->interpolateMotion(cnt);
            pos_x = (path_segment_buffer["x"]->d_pos) * w;
            if (pos_x == (pos_anterior_x + 1) || cnt >= path_segment_buffer["x"]->d_time)
            {
                motor_x->step();
                pos_anterior_x++;
                if (pos_anterior_x == path_segment_buffer["x"]->d_pos)
                {
                    finished = true;
                }
            }
        }

        if(path_segment_buffer["y"] != nullptr)
        {
            w = path_segment_buffer["y"]->interpolateMotion(cnt);
            pos_y = (path_segment_buffer["y"]->d_pos) * w;
            if (pos_y == (pos_anterior_y + 1) || cnt >= path_segment_buffer["y"]->d_time)
            {
                motor_y->step();
                pos_anterior_y++;
                if (pos_anterior_y == path_segment_buffer["y"]->d_pos)
                {
                    finished = true;
                }
            }
        }

        if(path_segment_buffer["z"] != nullptr)
        {
            w = path_segment_buffer["z"]->interpolateMotion(cnt);
            pos_z = (path_segment_buffer["z"]->d_pos) * w;
            if (pos_z == (pos_anterior_z + 1) || cnt >= path_segment_buffer["z"]->d_time)
            {
                motor_z->step();
                pos_anterior_z++;
                if (pos_anterior_z == path_segment_buffer["z"]->d_pos)
                {
                    finished = true;
                }
            }
        }
    }

} // namespace System::Robot
