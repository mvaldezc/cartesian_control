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

    enum class MotorType : uint8_t
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
            IMotor(MotorType motorType, char * motorId) : motorType(motorType) {
                strncpy(this->motorId, motorId, 8); // Add motor ID
            }
            virtual ~IMotor() = default;
            virtual void enableMotor() = 0;
            virtual void disableMotor() = 0;
            virtual void setDirection(MotorDirection direction) = 0;
            virtual void setHome() = 0;
            volatile bool emergency_stop_flag = false;
            volatile bool enabled_flag = false;
            const MotorType motorType;
            char motorId[8];
        
        protected:
            volatile MotorDirection direction = MotorDirection::CounterClockwise;
    };

} // namespace Motor
