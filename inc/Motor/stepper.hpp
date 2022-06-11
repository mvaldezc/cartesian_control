/***********************************************************************
 * @file	:	stepper.hpp
 * @brief 	:	Stepper Motor Library
 * 				Library to control a stepper motor
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>
#include "imotor.hpp"
#include "board.hpp"

#ifdef RASP_PICO
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#endif

namespace Motor {
    
    /**
     * @class Stepper
     * @brief Implementation of stepper motor driver.
     */
    class Stepper : public IMotor
    {
        public:

            /**
             * @brief Custom constructor
             * @param[in] motorId    Eight char length motor identifier
             * @param[in] numOfSteps Motor's number of steps per revolution
             * @param[in] pinDir     Board's pin connected to motor driver DIR
             * @param[in] pinStep    Board's pin connected to motor driver STEP
             * @param[in] pinEn      Board's pin connected to motor driver ENABLE
             */
            Stepper(uint_fast16_t numOfSteps, uint pinDir, uint pinStep, uint pinEn)
            : IMotor(MotorType::Stepper), numOfSteps(numOfSteps), pinDir(pinDir), pinStep(pinStep)
              , pinEn(pinEn) 
            {
                mutex_init(&mutex);     // Initialize mutex
                this->initPins();       // Setup board pins for motor control
            }

            ~Stepper() { this->disableMotor(); } // Custom destructor

            Stepper(const Stepper &) = delete; // Copy constructor

            Stepper & operator=(const Stepper &) = delete; //Copy assignment operator

            Stepper(Stepper &&) = default; // Move constructor

            /**
             * @brief Advance one step.
             * @param[in] pulse_width_us Period of the pulse. Time between rising and falling edge.
             * @return True if the step pulse was sent.
             */ 
            bool step(uint64_t pulse_width_us);

            /**
             * @brief Enable motor output.
             */
            void enableMotor() override;

            /**
             * @brief Disable motor output.
             */
            void disableMotor() override;

            /**
             * @brief Set motor's direction of rotation.
             */
            void setDirection(MotorDirection direction) override;

            /**
             * @brief Reset motor's position to zero.
             */
            void setHome() override;

            /**
             * @brief Get absolute position in steps, from home.
             */
            int_fast32_t getAbsPosition() override;

        private:

            /** 
             * @brief Setup board pins for motor control.
             */
            void initPins();

            // Motor state variables
            volatile int_fast32_t stepCntAbs = 0;
            volatile uint_fast16_t stepCntRel = 0;

            // Motor configuration variables
            const uint_fast16_t numOfSteps;
            const uint pinDir;
            const uint pinStep;
            const uint pinEn;

            // Motor's mutex
            mutex_t mutex;
    };

} // namespace Motor
