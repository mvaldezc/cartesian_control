/***********************************************************************
 * @file	:	imotor.hpp
 * @brief 	:	Motor Interface
 * 				Definition of a motor interface to be independent of
 *              motor type
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <stdint.h>
#include <string.h>


namespace Motor
{
    enum class MotorDirection : bool
    {
        Clockwise = 0,
        CounterClockwise = 1
    };

    enum class MotorType : uint8_t
    {
        Stepper = 0,
        Servo = 1
    };

    struct MotorInfo
    {
        MotorInfo(MotorType motorType, char* motorIdPtr) : motorType(motorType){
            strncpy(this->motorId, motorIdPtr, 8);
        }
        MotorType motorType;
        char motorId[8];
    };

    class IMotor
    {
        public:
            IMotor(MotorType motorType, char* motorId) : motorInfo(motorType, motorId){}
            virtual ~IMotor() = default;
            virtual void enableMotor() = 0;
            virtual void disableMotor() = 0;
            const MotorInfo motorInfo;
            volatile bool emergency_stop_flag = false;
            volatile bool enabled_flag = false;
            volatile MotorDirection direction = MotorDirection::Clockwise;
    };
}