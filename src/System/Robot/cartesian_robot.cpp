#include "cartesian_robot.hpp"

namespace System::Robot {

    void CartesianRobotClient::save_path_list(path_params_t * path_list_ptr)
    {
        path_list = path_list_ptr;
        next_path = path_list;
    }

    void CartesianRobotClient::set_trajectory_buffer() 
    {
        InterpolationFactory::create(path_segment_buffer["x"], static_cast<InterpolationType>(next_path->path_type), next_path->pos_x, (next_path->time) / 4000.0);
        InterpolationFactory::create(path_segment_buffer["y"], static_cast<InterpolationType>(next_path->path_type), next_path->pos_y, (next_path->time) / 4000.0);
        InterpolationFactory::create(path_segment_buffer["z"], static_cast<InterpolationType>(next_path->path_type), next_path->pos_z, (next_path->time) / 4000.0);
        set_next_motor_direction();
    }

    void CartesianRobotClient::set_next_motor_direction() {
        motor_x->setDirection(static_cast<MotorDirection>(next_path->dir_x));
        motor_y->setDirection(static_cast<MotorDirection>(next_path->dir_y));
        motor_z->setDirection(static_cast<MotorDirection>(next_path->dir_z));
    }

    void CartesianRobotClient::enable_motors() {
        motor_x->enableMotor();
        motor_y->enableMotor();
        motor_z->enableMotor();
    }

    void CartesianRobotClient::disable_motors() {
        motor_x->disableMotor();
        motor_y->disableMotor();
        motor_z->disableMotor();
    }

    void CartesianRobotClient::execute_routine(size_t list_size, path_params_t * path_list_ptr)
    {
        save_path_list(path_list_ptr);
        enable_motors();
        for (int i = 0; i < list_size; i++)
        {
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

    void CartesianRobotClient::clean_trajectory_buffer(){
        delete path_segment_buffer["x"];
        delete path_segment_buffer["y"];
        delete path_segment_buffer["z"];
        path_segment_buffer["x"] = nullptr;
        path_segment_buffer["y"] = nullptr;
        path_segment_buffer["z"] = nullptr;
        next_path++;
    }

    void CartesianRobotClient::move_motors()
    {
        cnt = (++k) * sampling_period_sec;
        
        w = path_segment_buffer["x"]->interpolateMotion(cnt);
        pos_x = (path_segment_buffer["x"]->d_pos) * w;
        // pos_y = (pos_final_y - pos_inicial_y) * w + pos_inicial_y;
        if (pos_x == (pos_anterior_x + 1) || cnt >= path_segment_buffer["x"]->d_time)
        {
            if(motor_x->getType() == MotorType::Stepper){
                static_cast<Motor::Stepper *>(motor_x.get())->step(step_pulse_width_us);
            }
            pos_anterior_x++;
            if (pos_anterior_x == path_segment_buffer["x"]->d_pos)
            {
                finished = true;
            }
        }

        w = path_segment_buffer["y"]->interpolateMotion(cnt);
        pos_y = (path_segment_buffer["y"]->d_pos) * w;
        if (pos_y == (pos_anterior_y + 1) || cnt >= path_segment_buffer["y"]->d_time)
        {
            if(motor_y->getType() == MotorType::Stepper){
                static_cast<Motor::Stepper *>(motor_y.get())->step(step_pulse_width_us);
            }
            pos_anterior_y++;
            if (pos_anterior_y == path_segment_buffer["y"]->d_pos)
            {
                finished = true;
            }
        }

        w = path_segment_buffer["z"]->interpolateMotion(cnt);
        pos_z = (path_segment_buffer["z"]->d_pos) * w;
        if (pos_z == (pos_anterior_z + 1) || cnt >= path_segment_buffer["z"]->d_time)
        {
            if(motor_z->getType() == MotorType::Stepper){
                static_cast<Motor::Stepper *>(motor_z.get())->step(step_pulse_width_us);
            }
            pos_anterior_z++;
            if (pos_anterior_z == path_segment_buffer["z"]->d_pos)
            {
                finished = true;
            }
        }
    }

    bool CartesianRobotClient::openLoopLimitCheck(IMotor * motor)
    {
        float absPos_cm = motor->getAbsPosition() * x_axis.motorResolution_um * um_to_cm_factor;
        if(absPos_cm  >= x_axis.maxAbsDist_cm 
            || absPos_cm <= x_axis.minAbsDist_cm)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


} // namespace System::Robot
