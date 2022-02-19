/***********************************************************************
 * @file	:	isr_sampling.hpp
 * @brief 	:	Interrupt Service Routine Sampling Library
 * 				Library to sample inside an interruption
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <stdio.h>
#define RASP_PICO
#ifdef RASP_PICO
#include "hardware/timer.h"
#endif

namespace Sampler {

    /** 
     * @brief Implementation of a periodic sampler using a timer interruption.
     * It executes a specified function each sampling time.
     * It ensures a timer delay < 5% otherwise returns an error.
     */
    class TimerIsrSampler
    {
        static constexpr double error_threshold_factor = 1.1;
        static constexpr unsigned int max_blocking_cycles = 10;

        public:
            TimerIsrSampler(uint64_t sampling_period_us) : sampling_period(sampling_period_us) {}

            #ifdef RASP_PICO
            struct repeating_timer sampling_timer;
            #endif

            volatile bool timer_error_flag = false; // flag for timer error

        private:
            const uint64_t sampling_period;
            volatile int k = 0;                     // discrete time counter

            volatile uint64_t previous_time_us = 0; // previous timestamp in microseconds
            volatile uint64_t dif_time_us = 0;      // time difference between current and previous timestamps
            volatile uint64_t current_time_us = 0;  // current time in microseconds
            volatile int isrBlockingCnt = 0;        // counter for timer delays

        public:

            /**
             * @brief Initilize repeating timer and timer interruption.
             * @param[in] irq_handler Function to call on each timer interruption.
             * @param[in] user_data Pointer to data to pass to the irq_handler.
             */
            void init(repeating_timer_callback_t irq_handler, void * user_data)
            {
                #ifdef RASP_PICO
                add_repeating_timer_us(-sampling_period, irq_handler, user_data, &sampling_timer);
                #endif
            }

            /**
             * @brief Cancel repeating timer and timer interruption.
             */
            void cancel(){
                #ifdef RASP_PICO
                cancel_repeating_timer(&sampling_timer);
                #endif
                isrBlockingCnt = 0;
            }

            /**
             * @brief Check if the timer interrupt handler is called within the deadlines.
             * @return True if timer has an acceptable period, with error < 5%.
             */
            inline bool isr_time_check()
            {
                // Increment dicrete time counter
                k++;

                // Get current time
                #ifdef RASP_PICO
                current_time_us = time_us_64();
                #endif

                // Update time variables
                dif_time_us = current_time_us - previous_time_us;
                previous_time_us = current_time_us;

                // Check if timer is going to slow, then return an error
                if (dif_time_us > sampling_period * error_threshold_factor)
                {
                    isrBlockingCnt++;
                    if (isrBlockingCnt > max_blocking_cycles)
                    {
                        timer_error_flag = true;
                        cancel_repeating_timer(&sampling_timer);
                        return false;
                    }
                }
                return true;
            }
    };

} // namespace Sampler
