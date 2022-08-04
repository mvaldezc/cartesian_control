#include "trajectory_gen.hpp"
#include "math.h"

namespace Algorithm::TrajectoryGeneration {

    //********************************** Linear Polynomial **********************************//

    LinearInterpolation::LinearInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        // Time scaling w[t] ∈ [0,1]
        //   w[t] = a0 + a1*t
        a[1] = (double) 1 / d_time;
    }

    double LinearInterpolation::interpolateMotion(double time)
    {
        return a[1] * time;
    }

    //********************************** Cubic Polynomial **********************************//

    CubicInterpolation::CubicInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        // Time scaling w[t] ∈ [0,1]
        //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3
        a[2] = (double) 3 / (d_time * d_time);
        a[3] = (double)-2 / (d_time * d_time * d_time);
    }

    double CubicInterpolation::interpolateMotion(double time)
    {
        return a[2] * time * time + a[3] * time * time * time;
    }

    //********************************** Quintic Polynomial **********************************//

    QuinticInterpolation::QuinticInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        // Time scaling w[t] ∈ [0,1]
        //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        a[5] = (double) 6 / (d_time * d_time * d_time * d_time * d_time);
        a[4] = (double)-5 / 2 * a[5] * d_time;
        a[3] = (double)10 / (d_time * d_time * d_time);
    }

    double QuinticInterpolation::interpolateMotion(double time)
    {
        return time * time * time * (a[3] + a[4] * time + a[5] * time * time);
    }

    //********************************** Septic Polynomial **********************************//

    SepticInterpolation::SepticInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        // Time scaling w[t] ∈ [0,1]
        //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7

        a[4] = (double) 35 / (d_time * d_time * d_time * d_time);
        a[5] = (double)-12 / 5 * a[4] / d_time;
        a[6] = (double) -5 / 6 * a[5] / d_time;
        a[7] = (double) -2 / 7 * a[6] / d_time;
    }

    double SepticInterpolation::interpolateMotion(double time)
    {
        return time * time * time * time * (a[4] + a[5] * time + a[6] * time * time + a[7] * time * time * time);
    }

    //******************************** Trapezoid Velocity Profile ********************************//

    TrapezoidInterpolation::TrapezoidInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        //    accel = 2*thb/tb^2
        //    vel = accel*tb
        //    tb = (2*tf*thb) / (2*thb+thf)

        if (d_pos >= 2000)
        {
            thb = (double)200;
        }
        else if (d_pos >= 1000)
        {
            thb = (double)100;
        }
        else if (d_pos >= 500)
        {
            thb = (double)50;
        }
        else if (d_pos >= 250)
        {
            thb = (double)25;
        }
        else
        {
            activate_linear = true;
            vel = (double)1 / d_time;
            return;
        }

        tb = (double)2 * thb * d_time / (2 * thb + d_pos);
        accel = (double)2 * thb / (tb * tb);
        vel = (double)2 * thb / tb;
        if (tb >= 0 && tb < d_time / 2 && accel > 4 * d_pos / (d_time * d_time) && vel < vel_max)
        {
            check_flag = true;
        }
    }

    double TrapezoidInterpolation::interpolateMotion(double time)
    {
        if (check_flag)
        {
            if (time < tb)
            {
                return_val = (double)1 / 2 * accel * time * time;
            }
            else if (time >= tb && time <= d_time - tb)
            {
                return_val = (double)-thb + vel * time;
            }
            else if (time > d_time - tb && time <= d_time)
            {
                return_val = (double)vel * d_time - accel * tb * tb - accel * (time - d_time) * (time - d_time) / 2;
            }
            else
            {
                return_val = (double)1;
            }
            return return_val / d_pos;
        }
        else if (activate_linear)
        {
            return vel * time;
        }
        return 0.0;
    }

    //****************************** Constant Velocity Septic Interpolation ******************************//

    SmoothInterpolation::SmoothInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        //  {(5 (2 qb + qf)^4)/(16 qb^3 tf^4)}, {-((3 (2 qb + qf)^5)/(16 qb^4 tf^5))}, {(2 qb + qf)^6/(32 qb^5 tf^6)}

        if (d_pos >= 1000)
        {
            thb = (double) 200;
        }
        else
        {
            activate_linear = true;
            vel = (double)1 / d_time;
            return;
        }
        double temp = (double)(2 * thb + d_pos) * (2 * thb + d_pos) * (2 * thb + d_pos) * (2 * thb + d_pos);
        double temp2 = (double)(16 * thb * thb * thb * d_time * d_time * d_time * d_time);
        a[4] = (double)5 * temp / temp2;
        a[5] = (double)-3 * temp * (2 * thb + d_pos) / (temp2 * thb * d_time);
        a[6] = (double)1 / 2 * temp * (2 * thb + d_pos) * (2 * thb + d_pos) / (temp2 * thb * d_time * thb * d_time);

        tb = (double)2 * thb * d_time / (2 * thb + d_pos);
        vel = (double)(d_pos - 2 * thb) / (d_time - 2 * tb);
        a[1] = vel;

        if (tb >= 0 && tb < d_time / 2 && vel < vel_max)
        {
            check_flag = true;
        }
    }

    double SmoothInterpolation::interpolateMotion(double time)
    {
        if (check_flag)
        {
            if (time < tb)
            {
                return_val = time * time * time * time * (a[4] + a[5] * time + a[6] * time * time);
            }
            else if (time >= tb && time <= d_time - tb)
            {
                return_val = (double)-thb + vel * time;
            }
            else if (time > d_time - tb && time <= d_time)
            {
                double time_c = time - d_time + tb;
                return_val = a[1] * time_c - time_c * time_c * time_c * time_c * (a[4] + a[5] * time_c + a[6] * time_c * time_c) + d_pos - thb;
            }
            else
            {
                return_val = (double)1;
            }
            return return_val / d_pos;
        }
        else if (activate_linear)
        {
            return vel * time;
        }
        return 0.0;
    }

    //****************************** Advanced Linear Interpolation ******************************//

    AdvancedInterpolation::AdvancedInterpolation(unsigned int delta_pos, double delta_time, JointType beginning, JointType ending,
        double vel_0, double vel_f) : ITrajectoryInterpolation(delta_pos, delta_time)
    {
        if(delta_pos == 0 || delta_time <= 0.0)
        {
            is_not_achievable = true;
            return;
        }
        if((double) delta_pos / delta_time > vel_max_abs || vel_0 > vel_max_abs || vel_f > vel_max_abs)
        {
            is_not_achievable = true;
            return;
        }

        if (beginning == JointType::Zero && ending == JointType::Zero)
        {   
            // Check accel_max is not overpassed.
            if((double) delta_pos > (double) accel_max_abs * delta_time * delta_time / 6)
            {
                activate_linear = true;
                vel_const = (double) 1 / delta_time;
                return;
            }
            
            volatile double temp = accel_max_abs * (-6 * (double) delta_pos + accel_max_abs * delta_time * delta_time);
            volatile double temp2 = sqrt(temp);
            vel_const = (double) 1/3 * (accel_max_abs * delta_time - sqrt(accel_max_abs * (-6 * (double) delta_pos + accel_max_abs * delta_time * delta_time)));
            qb = (double) 3 * (vel_const * vel_const) / (4 * accel_max_abs);
            tb = (double) 2 * qb / (vel_const);

            // Check vel_max is not overpassed.
            if(vel_const > vel_max_abs)
            {
                activate_linear = true;
                vel_const = (double) 1 / delta_time;
                return;
            }

            a[1] = vel_const;
            a[3] = (double) 4 * accel_max_abs * accel_max_abs / (9 * vel_const);
            a[4] = (double) - 4 * accel_max_abs * accel_max_abs * accel_max_abs / (27 * vel_const * vel_const);
        }
        else if(beginning == JointType::Continuous && ending == JointType::Continuous   ||
                beginning == JointType::Zero && ending == JointType::Continuous         ||
                beginning == JointType::Continuous && ending == JointType::Zero         )
        {
            /******************* Check interpolation direction *******************/
            /* Depending on whether vel_0 or vel_f is bigger, interpolation can change its direction.
             * For all of them, direction depends also on vconst value.
             *
             *   UpDown      UpUp    DownDown   DownUp
             *    _____                
             *   /     \     ____/    \____     \      /
             *  /       \   /              \     \____/
             * 
             * 
             */

            InterpType InterpolationDirection;
            double vel_min = 0, vel_max = 0, vel_mid = 0;
            double qlim_min = 0, qlim_max = 0, qlim_mid = 0;

            if(vel_f == vel_0) // Possibility UpDown and DownUp
            {
                vel_mid = vel_0;
                qlim_mid = delta_time * vel_mid;
                InterpolationDirection = (double) delta_pos > qlim_mid ? InterpolationDirection = InterpType::UpDown : InterpType::DownUp;
            }
            else // Possibility UpUp, DownDown, UpDown and DownUp
            {
                vel_max = vel_f > vel_0 ? vel_f : vel_0;
                vel_min = vel_f > vel_0 ? vel_0 : vel_f;
                qlim_min = delta_time * vel_min + 3 * (vel_0 - vel_f) * (vel_0 - vel_f) / (4 * accel_max_abs);
                qlim_max = delta_time * vel_max - 3 * (vel_0 - vel_f) * (vel_0 - vel_f) / (4 * accel_max_abs);

                if((double) delta_pos > qlim_max)
                {
                    InterpolationDirection = InterpType::UpDown;
                }
                else if((double) delta_pos < qlim_min)
                {
                    InterpolationDirection = InterpType::DownUp;
                }
                else // qlim_max > delta_pos > qlim_min
                {
                    InterpolationDirection = vel_f > vel_0 ? InterpType::UpUp : InterpType::DownDown;
                }
            } 

            /****************** Check accel and speed limits ******************/
            /* Accel limit
             * This implies checking max acceleration required does not overpass accel_max_abs.
             * At maximum acceleration, the vconst part of the interpolation disappears and 
             * speed plot becames a classical fifth grade interpolation. At a bigger delta_pos,
             * speed becames complex.
             * 
             * Speed limit
             * This implies checking max speed required does not overpass vel_max 
             * or downpass zero.
             * 
             * These only happens for UpDown and DownUp types.
             * 
             */

            double val_a1 = (double) accel_max_abs * delta_time * delta_time / 6;
            double val_a2 = (double) 3 * (vel_0 - vel_f) * (vel_0 - vel_f) / (8 * accel_max_abs);
            double val_a3 = (double) 1/2 * delta_time * (vel_0 + vel_f);

            double val_v1 = (double) 3 * (vel_0 * vel_0 + vel_f * vel_f) / (4 * accel_max_abs);
            double val_v2 = (double) delta_time * vel_max_abs;
            double val_v3 = (double) (6 * (vel_0 + vel_f) * vel_max_abs - 6 * vel_max_abs * vel_max_abs) / (4 * accel_max_abs);

            if(InterpolationDirection == InterpType::UpDown && (double) delta_pos > val_a1 - val_a2 + val_a3
            || InterpolationDirection == InterpType::DownUp && (double) delta_pos < - val_a1 + val_a2 + val_a3
            || InterpolationDirection == InterpType::UpDown  && (double) delta_pos > - val_v1 + val_v2 + val_v3 
            || InterpolationDirection == InterpType::DownUp && (double) delta_pos < val_v1)
            {
                    activate_linear = true;
                    vel_const = (double) 1 / delta_time;
                    return;
            }
        }        
    }

    double AdvancedInterpolation::interpolateMotion(double time)
    {
        if (!is_not_achievable)
        {
            if(activate_linear)
            {
                return vel_const * time;
            }
            if (time < tb)
            {
                return_val = time * time * time * (a[3] + a[4] * time);
            }
            else if (time >= tb && time <= d_time - tb)
            {
                return_val = (double) vel_const * (time - tb) + qb;
            }
            else if (time > d_time - tb && time <= d_time)
            {
                double time_c = time - d_time + tb;
                return_val = a[1] * time_c - time_c * time_c * time_c * (a[3] + a[4] * time_c) + d_pos - qb;
            }
            else
            {
                return_val = 0.0;
            }
            return return_val / d_pos;
        }
        return 0.0;
    }

    //************************************** Interpolation Factory **************************************//

    void InterpolationFactory::create(ITrajectoryInterpolation *&path_segment_ptr,
        InterpolationType path_type, unsigned int delta_pos, double delta_time)
    {
        // Allocate interpolation class in heap depending on interpolation type.
        switch (path_type)
        {
            case InterpolationType::LinearPoly:
                path_segment_ptr = new LinearInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::P2P_CubicPoly:
                path_segment_ptr = new CubicInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::P2P_QuinticPoly:
                path_segment_ptr = new QuinticInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::P2P_SepticPoly:
                path_segment_ptr = new SepticInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::P2P_TrapezoidPoly:
                path_segment_ptr = new TrapezoidInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::AdvancedPoly:
                path_segment_ptr = new AdvancedInterpolation(delta_pos, delta_time, 
                AdvancedInterpolation::JointType::Zero, AdvancedInterpolation::JointType::Zero, 0, 0);
                break;
        }
    }

} // namespace Algorithm::TrajectoryGeneration
