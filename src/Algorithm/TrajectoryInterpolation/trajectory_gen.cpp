#include "trajectory_gen.hpp"

namespace Algorithm {
namespace TrajectoryGeneration {

    //********************************** Linear Polynomial **********************************//

    LinearInterpolation::LinearInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time, 0, 0)
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
        : ITrajectoryInterpolation(delta_pos, delta_time, 0, 0)
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
        : ITrajectoryInterpolation(delta_pos, delta_time, 0, 0)
    {
        // Time scaling w[t] ∈ [0,1]
        //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        a[5] = (double)6 / (d_time * d_time * d_time * d_time * d_time);
        a[4] = (double)-5 / 2 * a[5] * d_time;
        a[3] = (double)10 / (d_time * d_time * d_time);
    }

    double QuinticInterpolation::interpolateMotion(double time)
    {
        return time * time * time * (a[3] + a[4] * time + a[5] * time * time);
    }

    //********************************** Septic Polynomial **********************************//

    SepticInterpolation::SepticInterpolation(unsigned int delta_pos, double delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time, 0, 0)
    {
        // Time scaling w[t] ∈ [0,1]
        //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7

        a[4] = (double)35 / (d_time * d_time * d_time * d_time);
        a[5] = (double)-12 / 5 * a[4] / d_time;
        a[6] = (double)-5 / 6 * a[5] / d_time;
        a[7] = (double)-2 / 7 * a[6] / d_time;
    }

    double SepticInterpolation::interpolateMotion(double time)
    {
        return time * time * time * time * (a[4] + a[5] * time + a[6] * time * time + a[7] * time * time * time);
    }

    //******************************** Trapezoid Velocity Profile ********************************//

    TrapezoidInterpolation::TrapezoidInterpolation(unsigned int delta_pos, unsigned int delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time, 0, 0)
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
        if (check_flag == true)
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

    SmoothInterpolation::SmoothInterpolation(unsigned int delta_pos, unsigned int delta_time)
        : ITrajectoryInterpolation(delta_pos, delta_time, 0, 0)
    {
        //  {(5 (2 qb + qf)^4)/(16 qb^3 tf^4)}, {-((3 (2 qb + qf)^5)/(16 qb^4 tf^5))}, {(2 qb + qf)^6/(32 qb^5 tf^6)}

        if (d_pos >= 2000)
        {
            thb = (double)400;
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
        if (check_flag == true)
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
                double temp = return_val * 2;
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

    void InterpolationFactory::create(ITrajectoryInterpolation *&path_segment_ptr,
        InterpolationType path_type, unsigned int delta_pos, double delta_time)
    {
        // Allocate interpolation class in heap depending on interpolation type.
        switch (path_type)
        {
            case InterpolationType::LinearPoly:
                path_segment_ptr = new LinearInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::CubicPoly:
                path_segment_ptr = new CubicInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::QuinticPoly:
                path_segment_ptr = new QuinticInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::SepticPoly:
                path_segment_ptr = new SepticInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::TrapezoidPoly:
                path_segment_ptr = new TrapezoidInterpolation(delta_pos, delta_time);
                break;
            case InterpolationType::SmoothPoly:
                path_segment_ptr = new SmoothInterpolation(delta_pos, delta_time);
                break;
        }
    }

} // namespace TrajectoryGeneration
} // namespace Algorithm
