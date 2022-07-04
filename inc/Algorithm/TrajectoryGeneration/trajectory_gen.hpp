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

    enum class InterpolationType
    {
        LinearPoly,
        CubicPoly,
        QuinticPoly,
        SepticPoly,
        TrapezoidPoly,
        SmoothPoly
    };

    /**
     * @interface ITrajectoryInterpolation
     * @brief Interface for motion description of a path segment.
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
            double interpolateMotion(double time) override;

        private:
            double a[2] = {0}; // Polynomial coefficients
    };

    /**
     * @brief Define a path segment with motion described by cubic polynomial.
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
     * @brief Define a path segment with motion described by quintic polynomial.
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
     * @brief Define a path segment with motion described by septic polynomial.
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

    /**
     * @brief Static class that manages trajectory interpolation types. Based on Factory Method design pattern.
     */
    class InterpolationFactory
    {
        public: 
            /**
             * @brief Prepares a specific trajectory interpolation type.
             * @param[in] path_type Type of the next required interpolation.
             * @param[in] delta_pos Amount of steps for next path_segment.
             * @param[in] delta_time Amount of time for next path_segment.
             * @return Pointer to trajectory interpolation that will be used in next path.
             */
            static std::unique_ptr<ITrajectoryInterpolation> create(
                InterpolationType path_type, unsigned int delta_pos, double delta_time);
        private:
            InterpolationFactory() = default;
    };

} // namespace Algorithm::TrajectoryGeneration
