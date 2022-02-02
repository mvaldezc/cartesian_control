#pragma once

// Time scaling w[t] ∈ [0,1]
//   w[t] = a0 + a1*t + a2*t^2 + a3*t^3
// Boundary values:
//   w[0] = 0, w[tf]=1, w'[0]=0, w'[tf]=0
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
// Since it is linear -> interpolation in task space is equivalent to joint space
//   θx[w[t]] = Px/α = (θxf - θxo) * w + θxo
//   θy[w[t]] = Py/β = (θyf - θyo) * w + θyo

namespace Algorithm {
namespace TrajectoryGeneration {

    enum class TrajectoryInterpolationType
    {
        linear_polynomial,
        cubic_polynomial,
        quintic_polynomial,
        septic_polynomial
    };

    typedef struct
    {
        TrajectoryInterpolationType path_type;
        unsigned int pos_i;
        unsigned int pos_f;
        unsigned int time_f;
    } path_t;

    class trajectory_segment_t
    {
        public:
            trajectory_segment_t(unsigned int pos_i, unsigned int pos_f, unsigned int time_interval_sec)
                : pos_i(pos_i), pos_f(pos_f), time_f(time_interval_sec) {}

            const unsigned int pos_i;
            const unsigned int pos_f;
            const unsigned int time_f;
            virtual double poly_move(double time) = 0;
    };

    class linear_segment_t : public trajectory_segment_t
    {
        public:
            double a[2];

            linear_segment_t(unsigned int pos_i, unsigned int pos_f, unsigned int time_interval_sec)
                : trajectory_segment_t(pos_i, pos_f, time_interval_sec) 
            {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t
                a[1] = (double)1 / time_f;
            }

            double poly_move(double time) override{
                return a[1] * time;
            }
    };

    class cubic_segment_t : public trajectory_segment_t
    {
        public:
            double a[4];

            cubic_segment_t(unsigned int pos_i, unsigned int pos_f, unsigned int time_interval_sec)
                : trajectory_segment_t(pos_i, pos_f, time_interval_sec)
            {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3
                a[2] = (double)  3 / (time_f * time_f);
                a[3] = (double) -2 / (time_f * time_f * time_f);
            }

            double poly_move(double time) override{
                // return a[2] * time * time + a[3] * time * time * time;
                return time * time * (a[2] + a[3] * time);
            }
    };

    class quintic_segment_t : public trajectory_segment_t
    {
        public:
            double a[6];

            quintic_segment_t(unsigned int pos_i, unsigned int pos_f, unsigned int time_interval_sec)
                : trajectory_segment_t(pos_i, pos_f, time_interval_sec)
            {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
                a[5] = (double) 6 / (time_f * time_f * time_f * time_f * time_f);
                a[4] = (double)-5 / 2 * a[5] * time_f;
                a[3] = (double) 5 / 3 * a[5] * (time_f * time_f);
            }

            double poly_move(double time) override{
                return time * time * time * (
                    a[3] 
                  + a[4] * time 
                  + a[5] * time * time
                );
            }
    };

    class septic_segment_t : public trajectory_segment_t
    {
        public:
            double a[8];

            septic_segment_t(unsigned int pos_i, unsigned int pos_f, unsigned int time_interval_sec)
                : trajectory_segment_t(pos_i, pos_f, time_interval_sec)
            {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
                a[4] = (double) 35 / (time_f * time_f * time_f * time_f);
                a[5] = (double)-12 / 5 * a[4] / time_f;
                a[6] = (double) -5 / 6 * a[5] / time_f;
                a[7] = (double) -2 / 7 * a[6] / time_f;
            }

            double poly_move(double time) override{
                return time * time * time * time * (
                    a[4] 
                  + a[5] * time 
                  + a[6] * time * time 
                  + a[7] * time * time * time
                );
            }
    };

    class trapezoid_segment_t : public trajectory_segment_t
    {
        public:
            trapezoid_segment_t(unsigned int pos_i, unsigned int pos_f, unsigned int time_interval_sec)
                : trajectory_segment_t(pos_i, pos_f, time_interval_sec)
            {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
                
            }

            double poly_move(double time) override
            {
            }

            double a[8];
    };

} // namespace TrajectoryGeneration
} // namespace Algorithm
