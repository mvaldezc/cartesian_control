/***********************************************************************
 * @file	:	trajectory_gen.hpp
 * @brief 	:	Trajectory Generation Library
 * 				Library to generate joint space trajectory interpolations
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once

// This library assumes joint interpolation is the same as task interpolation for cartesian robots.
// Proof:
//
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
//   θx[w[t]] = Px/α = (θxf - θxo) * w + θxo LinearPoly
//   θy[w[t]] = Py/β = (θyf - θyo) * w + θyo

namespace Algorithm {
namespace TrajectoryGeneration {

        enum class InterpolationType
        {
            LinearPoly,
            CubicPoly,
            QuinticPoly,
            SepticPoly,
            TrapezoidPoly,
            SmoothPoly
        };

        typedef struct // size: (3 words = 12 bytes = 96 bits)
        {
            unsigned int path_type : 5; // 32 types of movement
            unsigned int dir_x : 1;     // 0 is CCW, 1 is CW
            unsigned int dir_y : 1;     // 0 is CCW, 1 is CW
            unsigned int dir_z : 1;     // 0 is CCW, 1 is CW
            unsigned int time : 22;     // (1/4) ms, max 17 min for a single movement
            unsigned int : 0;           // 2 bit padding
            unsigned int pos_x : 16;    // steps, max 65535 steps or 320 revs
            unsigned int pos_y : 16;    // steps, max 65535 steps or 320 revs
            unsigned int pos_z : 16;    // steps, max 65535 steps or 320 revs
            unsigned int : 0;           // 16 bit padding
        } path_params_t;

        typedef struct // size: (6 words = 24 bytes = 112 bits)
        {
            unsigned int path_type : 5; // 32 types of movement
            unsigned int dir_x : 1;     // 0 is CCW, 1 is CW
            unsigned int dir_y : 1;     // 0 is CCW, 1 is CW
            unsigned int dir_z : 1;     // 0 is CCW, 1 is CW
            unsigned int time : 22;    // (1/4) ms, max 17 min for a single movement
            unsigned int : 0;          // 2 bit padding
            unsigned int pos_x : 16;   // steps, max 65535 steps or 320 revs
            unsigned int pos_y : 16;   // steps, max 65535 steps or 320 revs
            unsigned int pos_z : 16;   // steps, max 65535 steps or 320 revs
            unsigned int : 0;          // 16 bit padding
            unsigned int vel_x_0 : 16; // (1/20) step/sec, max 3200 steps/sec
            unsigned int vel_x_f : 16; // (1/20) step/sec, max 3200 steps/sec
            unsigned int vel_y_0 : 16; // (1/20) step/sec, max 3200 steps/sec
            unsigned int vel_y_f : 16; // (1/20) step/sec, max 3200 steps/sec
            unsigned int vel_z_0 : 16; // (1/20) step/sec, max 3200 steps/sec
            unsigned int vel_z_f : 16; // (1/20) step/sec, max 3200 steps/sec
        } path_params_extended_t;

        /**
         * @brief Interface for motion description of a path segment.
         */
        class ITrajectoryInterpolation {
            public:
                ITrajectoryInterpolation(unsigned int delta_pos, double delta_time, double vel_0, double vel_f)
                    : d_pos(delta_pos), d_time(delta_time), vel_0(vel_0), vel_f(vel_f) {}

                virtual ~ITrajectoryInterpolation() = default;
                virtual double interpolateMotion(volatile double time) = 0;

                const unsigned int d_pos;
                const double d_time;
                const double vel_0;
                const double vel_f;
        };

        /**
         * @brief Define a path segment with motion described by linear polynomial.
         * It allows to specify position constraints at boundary values of the segment.
         */ 
        class LinearInterpolation : public ITrajectoryInterpolation {
            public:
                LinearInterpolation(unsigned int delta_pos, double delta_time);

                /**
                 * @brief Calculates time scaling for parametric motion based on a linear polynomial.
                 * It achieves desired constraints (pos_0, pos_f) where pos_0 = 0.
                 * @param[in] time Time in seconds.
                 * @return Parameter w[t] = a0 + a1*t , w[t] ∈ [0,1]
                 */
                double interpolateMotion(volatile double time) override;

            private:
                double a[2]; // Polynomial coefficients
        };

        /**
         * @brief Define a path segment with motion described by cubic polynomial.
         * It allows to specify position and velocity constraints at boundary values of the segment.
         */
        class CubicInterpolation : public ITrajectoryInterpolation {
            public:
                CubicInterpolation(unsigned int delta_pos, double delta_time);

                /**
                 * @brief Calculates time scaling for parametric motion based on a third degree polynomial.
                 * It achieves desired constraints (pos_0, pos_f, vel_0, vel_f) where pos_0 = 0.
                 * @param[in] time Time in seconds.
                 * @return Parameter w[t] = a0 + a1*t + a2*t^2 + a3*t^3 , w[t] ∈ [0,1]
                 */
                double interpolateMotion(volatile double time) override;
            
            private:
                double a[4]; // Polynomial coefficients
        };

        /**
         * @brief Define a path segment with motion described by quintic polynomial.
         * It allows to specify position, velocity, and acceleration at boundary values of the segment.
         */
        class QuinticInterpolation : public ITrajectoryInterpolation {
            public:
                QuinticInterpolation(unsigned int delta_pos, double delta_time);

                /**
                 * @brief Calculates time scaling for parametric motion based on a fifth degree polynomial.
                 * It achieves desired constraints (pos_0, pos_f, vel_0, vel_f, accel_0, accel_f) 
                 * where pos_0 = accel_0 = accel_f = 0.
                 * @param[in] time Time in seconds.
                 * @return Parameter w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 , w[t] ∈ [0,1]
                 */
                double interpolateMotion(volatile double time) override;
            
            private:
                double a[6]; // Polynomial coefficients
        };

        /**
         * @brief Define a path segment with motion described by septic polynomial.
         * It allows to specify position, velocity, acceleration and jerk at boundary values of the segment.
         */
        class SepticInterpolation : public ITrajectoryInterpolation {
            public:
                SepticInterpolation(unsigned int delta_pos, double delta_time);

                /**
                 * @brief Calculates time scaling for parametric motion based on a seventh degree polynomial.
                 * It achieves desired constraints (pos_0, pos_f, vel_0, vel_f, accel_0, accel_f, jerk_0, jerk_f)
                 * where pos_0 = accel_0 = accel_f = jerk_0 = jerk_f = 0.
                 * @param[in] time Time in seconds.
                 * @return Parameter w[t] = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7 , w[t] ∈ [0,1]
                 */
                double interpolateMotion(volatile double time);

            private:
                double a[8]; // Polynomial coefficients
        };

        class TrapezoidInterpolation : public ITrajectoryInterpolation {
            public:
                TrapezoidInterpolation(unsigned int delta_pos, unsigned int delta_time);

                double interpolateMotion(volatile double time) override;
            
            private:
                double tb = 0;
                double accel = 0;
                double vel = 0;
                double vel_max = 2000;
                double return_val = 0;
                double thb = 0;
                bool check_flag = false;
                bool activate_linear = false;
        };

        class SmoothInterpolation : public ITrajectoryInterpolation {
            public:
                SmoothInterpolation(unsigned int delta_pos, unsigned int delta_time);

                double interpolateMotion(volatile double time) override;

            private:
                double a[8];
                double tb = 0;
                double vel = 0;
                double vel_max = 2000;
                double return_val = 0;
                double thb = 0;
                bool check_flag = false;
                bool activate_linear = false;
        };

} // namespace TrajectoryGeneration
} // namespace Algorithm
