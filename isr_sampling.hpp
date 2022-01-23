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

namespace Sample
{
    typedef struct
    {
        volatile uint64_t previous_time_us = 0;  // previous timestamp in microseconds
        volatile uint64_t dif_time_us = 0;       // time difference between current and previous timestamps
        volatile uint64_t current_time_us = 0;   // current time in microseconds
        volatile int k = 0;                      // discrete time counter
        volatile int isrBlockingCnt = 0;         // counter for timer delays
        volatile bool timer_error_flag = false;  // flag for timer error
    } ISR_SamplingData_t;

    class TimerIsrSampler
    {
        public:
            TimerIsrSampler(uint64_t sampling_time_us, int max_blocking_cycles, double error_threshold_factor)
                : sampling_time_us(sampling_time_us), max_blocking_cycles(max_blocking_cycles)
                , error_threshold_factor(error_threshold_factor) {}

            ISR_SamplingData_t isr_data;

            #ifdef RASP_PICO
            struct repeating_timer sampling_timer;
            #endif

        private:
            const uint64_t sampling_time_us;
            const int max_blocking_cycles;
            const double error_threshold_factor;

        public:

            /**
             * @brief Initilize repeating timer and timer interruption.
             * @param[in] irq_handler Function to call on each timer interruption.
             * @param[in] user_data Pointer to data to pass to the irq_handler.
             */
            void init(repeating_timer_callback_t irq_handler, void * user_data)
            {
                #ifdef RASP_PICO
                add_repeating_timer_us(-sampling_time_us, irq_handler, user_data, &sampling_timer);
                #endif
            }

            /**
             * @brief Cancel repeating timer and timer interruption.
             */
            void cancel(){
                cancel_repeating_timer(&sampling_timer);
            }

            /**
             * @brief Check if the timer interrupt handler is called within the deadlines.
             * @return True if timer has an acceptable period.
             */
            inline bool isr_time_check()
            {
                // Increment dicrete time counter
                isr_data.k++;

                // Get current time
                #ifdef RASP_PICO
                isr_data.current_time_us = time_us_64();
                #endif

                // Update time variables
                isr_data.dif_time_us = isr_data.current_time_us - isr_data.previous_time_us;
                isr_data.previous_time_us = isr_data.current_time_us;

                // Check if timer is going to slow, then return an error
                if (isr_data.dif_time_us > sampling_time_us * error_threshold_factor)
                {
                    isr_data.isrBlockingCnt++;
                    if (isr_data.isrBlockingCnt > max_blocking_cycles)
                    {
                        isr_data.timer_error_flag = true;
                        return false;
                    }
                }
                return true;
            }
    };

}


