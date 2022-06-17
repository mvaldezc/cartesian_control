/***********************************************************************
 * @file	:	isr_sampling.hpp
 * @brief 	:	Interrupt Service Routine Sampling Library
 * 				Library to sample using a timer interruption.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdio>

#ifdef RASP_PICO
#include "hardware/timer.h"
#endif

namespace Sampler {

    /**
     * @class TimerIsrSampler
     * @brief Implementation of a periodic sampler using a timer interruption.
     * It executes a specified function each sampling time.
     * It checks if timer delay < 5% otherwise returns an error and stops.
     * Sampling period range :  100 us  < T < 17 min
     */
    class TimerIsrSampler
    {
        static constexpr double ERROR_THRESHOLD_FACTOR = 1.1;
        static constexpr unsigned int MAX_BLOCKING_CYCLES = 10;

        public:
            TimerIsrSampler(uint64_t sampling_period_us) : samplingPeriod(sampling_period_us) {}

            #ifdef RASP_PICO
            struct repeating_timer sampling_timer;
            #endif

            volatile bool errorFlag = false; // flag for timer error

        private:
            const uint64_t samplingPeriod;
            volatile uint64_t currentTime_us = 0;  // current timestamp in microseconds
            volatile uint64_t previousTime_us = 0; // previous timestamp in microseconds
            volatile uint64_t deltaTime_us = 0;      // time difference between current and previous timestamps
            volatile int isrBlockingCnt = 0;        // counter for timer delays

        public:

            /**
             * @brief Initilize repeating timer and timer interruption.
             * @param[in] irq_handler Function to call on each timer interruption.
             * @param[in] user_data Pointer to data to pass to the irq_handler.
             */
            inline void init(repeating_timer_callback_t irq_handler, void * user_data)
            {
                #ifdef RASP_PICO
                add_repeating_timer_us(-samplingPeriod, irq_handler, user_data, &sampling_timer);
                #endif
            }

            /**
             * @brief Cancel repeating timer and timer interruption.
             */
            inline void cancel(){
                #ifdef RASP_PICO
                cancel_repeating_timer(&sampling_timer);
                #endif
                isrBlockingCnt = 0;
            }

            /**
             * @brief Check if the timer interrupt handler is called within the deadlines.
             * @return True if timer has an acceptable period, with error < 5%.
             */
            inline bool isrTimeCheck()
            {
                // Get current time
                #ifdef RASP_PICO
                currentTime_us = time_us_64();
                #endif

                // Update time variables
                deltaTime_us = currentTime_us - previousTime_us;
                previousTime_us = currentTime_us;

                // Check if timer is going to slow, then return an error
                if (deltaTime_us > samplingPeriod * ERROR_THRESHOLD_FACTOR)
                {
                    isrBlockingCnt++;
                    if (isrBlockingCnt > MAX_BLOCKING_CYCLES)
                    {
                        errorFlag = true;
                        cancel_repeating_timer(&sampling_timer);
                        return false;
                    }
                }
                return true;
            }
    };

} // namespace Sampler
