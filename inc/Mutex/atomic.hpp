#pragma once
#include "pico/mutex.h"
#include "lock_guard.hpp"

/**
 * @class atomic_bool
 * @brief Custom implementation of an atomic bool using Raspberry Pico's mutex api
 */
class atomic_bool
{
    private:
        bool flag;
        mutex_t mutex;

    public:
        atomic_bool() noexcept : atomic_bool(false) {};         // Custom default constructor

        atomic_bool(bool value) noexcept : flag(value)           // Initialization constructor
        {
            mutex_init(&mutex);
        }

        ~atomic_bool() noexcept = default;                            // Default destructor
        atomic_bool(const atomic_bool & obj) = delete;                // Delete copy constructor
        atomic_bool& operator=(const atomic_bool & obj) = delete;     // Delete copy assignment operator
        atomic_bool& operator=(const atomic_bool&) volatile = delete; // Delete copy assignment operator for volatile

        bool operator=(bool value) noexcept                     // Override asignment operator
        { 
            lock_guard<mutex_t> guard(mutex);
            flag = value;
            return value;
        }

        operator bool() noexcept                                // Override bool operator
        {
            lock_guard<mutex_t> guard(mutex);
            return flag;
        }

};
