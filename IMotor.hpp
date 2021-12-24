/***********************************************************************
 * @file	:	IMotor.hpp
 * @brief 	:	Motor Interface
 * 				Definition of a motor interface to be independent of
 *              motor type
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>

namespace MOTOR
{
    enum class MotorType : uint8_t
    {
        Stepper = 0,
        Servo = 1
    };

    struct MotorInfo
    {
        MotorInfo(MotorType motorType) : motorType(motorType){}
        MotorType motorType;
    };

    class IMotor
    {
    public:
        IMotor(MotorType motorType) : motorInfo(motorType){}
        virtual ~IMotor() = default;
        virtual void enableMotor() = 0;
        virtual void disableMotor() = 0;
        const MotorInfo motorInfo;
    };
}