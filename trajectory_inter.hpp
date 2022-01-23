#pragma once

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
        int pos_i;
        int pos_f;
        unsigned int time_interval;
    } path_t;

    class trajectory_segment_t
    {
        public:
            trajectory_segment_t(int pos_i, int pos_f, unsigned int time_interval)
                : pos_i(pos_i), pos_f(pos_f), time_interval(time_interval) {}

            const int pos_i;
            const int pos_f;
            const unsigned int time_interval;
            virtual double poly_move(double time) = 0;
    };

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

    class linear_segment_t : public trajectory_segment_t
    {
        public:
            linear_segment_t(int pos_i, int pos_f, unsigned int time_interval)
            : trajectory_segment_t(pos_i, pos_f, time_interval) {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t
                a[1] = (double)1 / time_interval;
            }

            double poly_move(double time) override{
                return a[1] * time;
            }

            double a[2];
    };

    class cubic_segment_t : public trajectory_segment_t
    {
        public:
            cubic_segment_t(int pos_i, int pos_f, unsigned int time_interval)
            : trajectory_segment_t(pos_i, pos_f, time_interval) {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3
                a[2] = (double)3 / (double)(time_interval * time_interval);
                a[3] = (double)-2 / (double)(time_interval * time_interval * time_interval);
            }

            double poly_move(double time) override{
                return a[2] * time * time + a[3] * time * time * time;
            }

            double a[4];
    };

    class quintic_segment_t : public trajectory_segment_t
    {
        public:
            quintic_segment_t(int pos_i, int pos_f, unsigned int time_interval)
            : trajectory_segment_t(pos_i, pos_f, time_interval) {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
                a[5] = (double)6 / (time_interval * time_interval * time_interval * time_interval * time_interval);
                a[4] = (double)-5 / 2  * a[5] * time_interval;
                a[3] = (double)5 / 3 * a[5] * (time_interval * time_interval);
            }

            double poly_move(double time) override{
                return 
                  a[3] * time * time * time 
                + a[4] * time * time * time * time 
                + a[5] * time * time * time * time * time;
            }

            double a[6];
    };

    class septic_segment_t : public trajectory_segment_t
    {
        public:
            septic_segment_t(int pos_i, int pos_f, unsigned int time_interval)
                : trajectory_segment_t(pos_i, pos_f, time_interval)
            {
                // Time scaling w[t] ∈ [0,1]
                //   w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
                a[4] = (double)35 / (time_interval * time_interval * time_interval * time_interval);
                a[5] = (double)-12 / 5 * a[4] / time_interval;
                a[6] = (double)-5 / 6 * a[5] / time_interval;
                a[7] = (double)-2 / 7 * a[6] / time_interval;
            }

            double poly_move(double time) override{
                return 
                  a[4] * time * time * time * time 
                + a[5] * time * time * time * time * time 
                + a[6] * time * time * time * time * time * time 
                + a[7] * time * time * time * time * time * time * time;
            }

            double a[8];
    };
}
}
