/***********************************************************************
 * @file	:	Stepper.hpp
 * @brief 	:	Stepper Motor Library
 * 				Library to control a stepper motor
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>
#include "IMotor.hpp"

namespace MOTOR
{
    class Stepper : IMotor
    {
        public:

            /**
             * @brief Custom constructor
             * @param[in] numOfSteps Motor's number of steps per revolution
             * @param[in] pinDir Board's pin connected to motor driver DIR
             * @param[in] pinStep Board's pin connected to motor driver STEP
             */
            Stepper(int numOfSteps, int pinDir, int pinStep) 
            : IMotor(MotorType::Stepper), numOfSteps(numOfSteps), pinDir(pinDir), pinStep(pinStep){} 

            ~Stepper() { disableMotor(); } // Custom destructor

            Stepper(const Stepper &) = delete; // Copy constructor

            Stepper & Stepper::operator=(const Stepper &) = delete; //Copy assignment operator

            void step();

            void enableMotor() override;

            void disableMotor() override;

        private:
            // Motor state
            uint8_t direction;
            uint32_t stepCntRel;
            uint32_t stepCntAbs;

            // Motor configuration variables
            const uint_fast16_t numOfSteps;
            const int pinDir;
            const int pinStep;
    };

} // MOTOR
