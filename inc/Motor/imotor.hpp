/***********************************************************************
 * @file	:	imotor.hpp
 * @brief 	:	Motor Interface
 * 				Definition of a motor interface to be independent of
 *              motor type
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>
#include <string.h>

namespace Motor {
    
    enum class MotorDirection : bool
    {
        CounterClockwise = 0,
        Clockwise = 1
    };

    enum class MotorType
    {
        Stepper = 0,
        Servo = 1
    };

    /**
     * @interface IMotor
     * @brief Motor driver interface.
     */
    class IMotor
    {
        public:
            IMotor(MotorType motorType) : motorType(motorType) {}
            virtual ~IMotor() = default;
            virtual void enableMotor() = 0;
            virtual void disableMotor() = 0;
            virtual void setDirection(MotorDirection direction) = 0;
            virtual void setHome() = 0;
            virtual int_fast32_t getAbsPosition() = 0;

            bool isEnabled()
            {
                return enabledFlag;
            }

            MotorType getType()
            {
                return motorType;
            }

        protected:
            volatile bool enabledFlag = false;
            volatile MotorDirection direction = MotorDirection::CounterClockwise;
            const MotorType motorType;
    };

} // namespace Motor
