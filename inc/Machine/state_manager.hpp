#pragma once

#include <string>
#include <cstdint>
#include "pico/critical_section.h"
#include "pico/mutex.h"
#include "lock_guard.hpp"

/* ---------------------------------------------------------------------------------------------- */
/*                                      Public Access Members                                     */
/* ---------------------------------------------------------------------------------------------- */

#define EMERGENCY_ENABLED (uint32_t)(1 << 8)
#define ACTION_NOT_NEEDED (uint32_t)(1 << 16)

enum class Action : uint8_t
{
    Cancel = 0x00,
    Start = 0x01,
    Next = 0x02,
    Done = 0x03,
    Jog = 0x04,
    Program = 0x05,
    Data = 0x06,
    None = 0xFF
};

enum class MachineState : uint8_t
{
    Off = 0x00,
    PrepareMove = 0x01,
    WaitStart = 0x02,
    ExecuteProgram = 0x03,
    Jog = 0x04,
    EmergencyStop = 0xFF
};

typedef union
{
    struct
    {
        volatile MachineState mode;
        volatile bool emergencyStop;
        volatile bool actionRequired;
    } state;
    volatile uint32_t key;
} MachineData;

// equivalent to static mutex declaration + mutex_init()
auto_init_mutex(createStateManagerMutex);

/**
 * @class StateManager
 * @brief Manages the machine state transitions of a Welding Corner Machine.
 * StateManager is defined as a singleton and provides a thread/ISR-safe API for 
 * triggering machine state transitions.
 * 
 * @details
 * It implements a static mutex to avoid race conditions during singleton creation.
 * And a critical_section for concurrent access of its non-reentrant methods.
 * 
 */
class StateManager
{   
    protected:

        StateManager()
        {
            critical_section_init(&stateManagerLock);
        }

        Action instructionBuffer = Action::None;
        MachineData machineData =
        {
            .state =
            {
                .mode = MachineState::Off,
                .emergencyStop = false,
                .actionRequired = false,
            }
        };
        critical_section_t stateManagerLock;

        bool dataPending = false;

    private: 
        static StateManager * instance;

    public:
        static StateManager * getInstance()
        {
            // Acquire mutex to avoid creation of multiple instances of class 
            // by concurrent calls of this method.
            lock_guard<mutex_t> mutexWrapper(createStateManagerMutex);

            // If unique StateManager instance doesn't exist, create it.
            if(instance == nullptr)
            {
                instance = new StateManager;
            }

            // Return singleton.
            return instance;
        }

        // Singleton shouldn't be cloneable.
        StateManager(const StateManager & obj) = delete;

        // Singleton shouldn't be assignable.
        StateManager & operator=(const StateManager & obj) = delete;

        void machineProcess();

        void setAction(Action instruction);

        void setEmergencyStop();

        MachineState getMachineState();

        bool isDataPending();

};
