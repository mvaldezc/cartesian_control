/***********************************************************************
 * @file	:	trajectory_gen.hpp
 * @brief 	:	Trajectory Generation Library
 * 				Library to generate joint space trajectory interpolations.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <memory>

// This library assumes joint interpolation is the same as task interpolation for cartesian robots.
// Example:
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
//   θx[w[t]] = Px/α = (θxf - θxo) * w + θxo 
//   θy[w[t]] = Py/β = (θyf - θyo) * w + θyo

namespace Algorithm::TrajectoryGeneration {

    /**
     * @enum InterpolationType
     * @brief Specifies the motion profile used for a desired path.
     */
    enum class InterpolationType
    {
        LinearPoly = 0,         ///< Linear path shape.
        P2P_CubicPoly = 1,      ///< Cubic polynomial time scaling (point to point behavior).
        P2P_QuinticPoly = 2,    ///< Quintic polynomial time scaling (point to point behavior).
        P2P_SepticPoly = 3,     ///< Septic polynomial time scaling (point to point behavior).
        P2P_TrapezoidPoly = 4,  ///< Trapezoid speed motion profile (point to point behavior).
        AdvancedPoly = 5        ///< S-curve speed motion profile (speed to speed behavior).
    };

    /**
     * @interface ITrajectoryInterpolation
     * @brief Interface for motion profile generators.
     */
    class ITrajectoryInterpolation {
        public:
            ITrajectoryInterpolation(unsigned int delta_pos, double delta_time)
                : d_pos(delta_pos), d_time(delta_time) {}

            virtual ~ITrajectoryInterpolation() = default;
            virtual double interpolateMotion(double time) = 0;

            const unsigned int d_pos;
            const double d_time;
    };

    /**
     * @brief Defines a path segment with motion described by linear polynomial time scaling.
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
            double interpolateMotion(double time) override;

        private:
            double a[2] = {0}; // Polynomial coefficients
    };

    /**
     * @brief Defines a path segment with motion described by cubic polynomial time scaling.
     * It allows to specify position constraints at boundary values of the segment.
     * Point-to-Point behavior so velocity at boundary values is set to zero.
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
            double interpolateMotion(double time) override;
        
        private:
            double a[4] = {0}; // Polynomial coefficients
    };

    /**
     * @brief Define a path segment with motion described by quintic polynomial time scaling.
     * It allows to specify position at boundary values of the segment.
     * Point-to-Point behavior so velocity and acceleration at boundary values
     * are set to zero.
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
            double interpolateMotion(double time) override;
        
        private:
            double a[6] = {0}; // Polynomial coefficients
    };

    /**
     * @brief Define a path segment with motion described by septic polynomial time scaling.
     * It allows to specify position at boundary values of the segment.
     * Point-to-Point behavior so velocity, acceleration and jerk at boundary values
     * are set to zero.
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
            double interpolateMotion(double time) override;

        private:
            double a[8] = {0}; // Polynomial coefficients
    };

    class TrapezoidInterpolation : public ITrajectoryInterpolation {
        public:
            TrapezoidInterpolation(unsigned int delta_pos, double delta_time);

            double interpolateMotion(double time) override;
        
        private:
            double tb = 0;
            double accel = 0;
            double vel = 0;
            double vel_max = 2000;
            double return_val = 0;
            double thb = 0;
            bool check_flag = false;
            bool activate_linear = false;
            bool other_category = false;
    };

    class SmoothInterpolation : public ITrajectoryInterpolation {
        public:
            SmoothInterpolation(unsigned int delta_pos, double delta_time);

            double interpolateMotion(double time) override;

        private:
            double a[8] = {0};
            double tb = 0;
            double vel = 0;
            double vel_max = 2000;
            double return_val = 0;
            double thb = 0;
            bool check_flag = false;
            bool activate_linear = false;
    };

    class AdvancedInterpolation : public ITrajectoryInterpolation {
        
        enum class InterpType {UpDown, UpUp, DownDown, DownUp};

        public:
            AdvancedInterpolation(unsigned int delta_pos, double delta_time, double vel_0, double vel_f);

            double interpolateMotion(double time) override;

        private:
            const double vel_max_abs = 2000; // [steps/s]
            const double accel_max_abs = 1500; // [steps/s^2]

            double a[6] = {0};
            double a_0[6] = {0};
            double a_f[6] = {0};
            double tb = 0, tb_0 = 0, tb_f = 0;
            double qb = 0, qb_0 = 0, qb_f = 0;
            double vel_const = 0, vel_0, vel_f;
            double v_start_0 = 0, v_end_0 = 0, v_start_f = 0, v_end_f = 0;
            double v_max_0 = 0, v_min_0 = 0, v_max_f = 0, v_min_f = 0;
            double return_val = 0;
            bool activate_linear = false;
            bool is_not_achievable = false;
    };

    /**
     * @brief Static class that manages trajectory interpolation types. Based on Factory Method design pattern.
     */
    class InterpolationFactory
    {
        public: 
            /**
             * @brief Prepares a specific trajectory interpolation type.
             * @param[out] path_segment_ptr Pointer to trajectory interpolation that will be used in next path.
             * @param[in] path_type Type of the next required interpolation.
             * @param[in] delta_pos Amount of steps for next path_segment.
             * @param[in] delta_time Amount of time for next path_segment.
             */
            static void create(ITrajectoryInterpolation * & path_segment_ptr,
                InterpolationType path_type, unsigned int delta_pos, double delta_time);
        private:
            InterpolationFactory() = default;
    };

} // namespace Algorithm::TrajectoryGeneration
