/***********************************************************************
 * @file	:	Stepper.hpp
 * @brief 	:	Stepper Motor Library
 * 				Library to control a stepper motor
 * @author	:	Marco Valdez
 *
 ***********************************************************************/

#pragma once
#include <cstdint>

namespace Motor{

class Stepper
{
    public:
        Stepper(int numOfSteps, int pinDir, int pinStep) 
         : numOfSteps(numOfSteps), pinDir(pinDir), pinStep(pinStep) {} // Constructor

        virtual ~Stepper() { disableMotor(); } // Destructor

        Stepper(const Stepper &) = delete; // Copy constructor
        
        Stepper & Stepper::operator=(const Stepper &) = delete; // Copy assignment operator

        void step();

        void enableMotor();

        void disableMotor();

    private:
        // Public 
        uint8_t direction;
        uint32_t stepCnt;

        // Motor configuration variables
        uint_fast16_t numOfSteps;
        int pinDir;
        int pinStep;

};

} // Motor
