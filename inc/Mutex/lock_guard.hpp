#pragma once
#include "pico/mutex.h"

/**
 * @class lock_guard
 * @brief Custom RAII mutex wrapper implementation
 * @tparam Mutex Any mutex class following Raspberry Pico's mutex api
 */
template<typename Mutex>
class lock_guard
{
    public:
        // Class constructor that acquires the mutex
        explicit lock_guard(Mutex & m) : m_(m)
        {
            mutex_enter_blocking(&m_);
        }

        // Release the mutex in destructor
        ~lock_guard()
        {
            mutex_exit(&m_);
        }

        // No copy constructor
        lock_guard(const lock_guard & obj) = delete;

        // No copy assignment operator
        lock_guard & operator=(const lock_guard & obj) = delete;

    private:

        // Mutex to call
        Mutex & m_;
};
