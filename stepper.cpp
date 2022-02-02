#include "stepper.hpp"

namespace Motor
{

    bool Stepper::step(uint64_t pulse_width_us)
    {
        // Return if motor is not enabled or emergency stop happened
        if (!enabled_flag || emergency_stop_flag){
            return false;
        }

        // Acquire the mutex
        uint32_t *mutex_owner;
        if (mutex_try_enter(&mutex, mutex_owner)){
            //  Execute step
            #ifdef RASP_PICO
            gpio_put(pinStep, 1);
            busy_wait_us(pulse_width_us);
            gpio_put(pinStep, 0);
            #endif

            // Update step counters
            if (direction == MotorDirection::CounterClockwise){
                stepCntAbs++;
                if (stepCntRel == numOfSteps - 1){
                    stepCntRel = 0;
                }
                else{
                    stepCntRel++;
                }
            }
            else{
                stepCntAbs--;
                if (stepCntRel == 0){
                    stepCntRel = numOfSteps - 1;
                }
                else{
                    stepCntRel--;
                }
            }
            // Liberar el mutex
            mutex_exit(&mutex);
            return true;
        }
        return false;
    }

    void Stepper::enableMotor(){
        #ifdef RASP_PICO
        // Set enabled flag in motor config register
        enabled_flag = true;
        // Atomic bit set of enable pin, send enable signal to motor
        gpio_put(pinEn, 0);
        #endif
    }

    void Stepper::disableMotor(){
        #ifdef RASP_PICO
        // Clear enabled flag in motor config register
        enabled_flag = false;
        // Atomic bit clear of enable pin, send disable signal to motor
        gpio_put(pinEn, 1);
        #endif
    }

    void Stepper::initPins(){
        gpio_init(pinEn);
        gpio_init(pinDir);
        gpio_init(pinStep);
        gpio_put(pinEn, 1);
        gpio_put(pinDir, 0);
        gpio_put(pinStep, 0);
        gpio_set_dir(pinEn, GPIO_OUT);
        gpio_set_dir(pinDir, GPIO_OUT);
        gpio_set_dir(pinStep, GPIO_OUT);
    }

    void Stepper::setDirection(MotorDirection direction){
        this->direction = direction;
        gpio_put(pinDir, static_cast<bool>(direction));
    }
}