/***********************************************************************
 * @file	:	stepper.hpp
 * @brief 	:	Stepper Motor Library
 * 				Library to control a stepper motor
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <stdint.h>
#include "imotor.hpp"

#include "pico/mutex.h"
#define RASP_PICO
#ifdef RASP_PICO
#include "hardware/gpio.h"
#include "hardware/timer.h"
#endif

namespace Motor
{
    class Stepper : IMotor
    {
        public:

            /**
             * @brief Custom constructor
             * @param[in] motorId Eight char length motor identifier
             * @param[in] numOfSteps Motor's number of steps per revolution
             * @param[in] pinDir Board's pin connected to motor driver DIR
             * @param[in] pinStep Board's pin connected to motor driver STEP
             */
            Stepper(char* motorId, uint_fast16_t numOfSteps, uint pinDir, uint pinStep, uint pinEn)
            : IMotor(MotorType::Stepper, motorId), numOfSteps(numOfSteps), pinDir(pinDir), pinStep(pinStep)
            , pinEn(pinEn) {
                mutex_init(&mutex);
                this->init();
            }

            ~Stepper() { this->disableMotor(); } // Custom destructor

            Stepper(const Stepper &) = delete; // Copy constructor

            Stepper & operator=(const Stepper &) = delete; //Copy assignment operator

            bool step(uint64_t pulse_width_us);

            void enableMotor() override;

            void disableMotor() override;

            void init();

        private:
            uint_fast16_t stepCntRel = 0;
            int32_t stepCntAbs = 0;

            // Motor configuration variables
            const uint_fast16_t numOfSteps;
            const uint pinDir;
            const uint pinStep;
            const uint pinEn;

            // Motor's exclusive access
            mutex_t mutex;
    };

} // MOTOR
