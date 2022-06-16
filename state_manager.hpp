#pragma once

#include <unordered_map>
#include <string>
#include <cstdint>
#include "trajectory_gen.hpp"
#include "pico/critical_section.h"
#include "pico/mutex"

/* ---------------------------------------------------------------------------------------------- */
/*                                      Public Access Members                                     */
/* ---------------------------------------------------------------------------------------------- */

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
        volatile bool actionNotRequired;
    } state;
    volatile uint32_t key;
} MachineData;

/**
 * @class StateManager
 * @brief Manages the machine state transitions of a Welding Corner Machine.
 * StateManager is defined as a singleton and provides a thread/ISR-safe API for 
 * triggering machine state transitions.
 * 
 * It implements a static mutex to avoid race conditions during singleton creation
 * It implements a critical_section for concurrent access of its non-reentrant methods
 */
class StateManager
{   
    protected:
        StateManager()
        {
            critical_section_init(*machineDataLock)
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
        critical_section_t stateDataLock;

    private: 
        static StateManager* stateManagerInstance;
        // equivalent to static mutex declaration + mutex_init()
        auto_init_mutex(createStateManagerMutex);
        

    public:
        static StateManager * getInstance()
        {
            // Acquire mutex to avoid creation of multiple instances of class 
            // by concurrent calls of this method.
            lock_guard<mutex_t> mutexWrapper(createStateManagerMutex);

            // If unique StateManager instance doesn't exist, create it.
            if(stateManagerInstance == nullptr)
            {
                stateManagerInstance = new StateManager;
            }

            // Return singleton.
            return stateManagerInstance;
        }

        // Singleton shouldn't be cloneable.
        StateManager(const StateManager & obj) = delete;

        // Singleton shouldn't be assignable.
        StateManager & operator=(const StateManager & obj) = delete;

        #define EMERGENCY_ENABLED (uint32_t)(1 << 8)
        #define ACTION_NOT_NEEDED (uint32_t)(1 << 16)

        void machineProcess()
        {
            critical_section_enter_blocking(&stateManagerLock);

            // solo si action needed hacer algo
            // solo si paro de emergencia ignorar el resto
            switch (machineData.key)
            {
                case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::Off)):
                case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::PrepareMove)):
                case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::WaitStart)):
                case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::ExecuteProgram)):
                case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::Jog)):
                    machineData.state.mode = MachineState::EmergencyStop;
                    printf("EMERGENCY STOP\n");
                    break;
                
                case (static_cast<uint32_t>(MachineState::Off)):
                    switch (instructionBuffer)
                    {
                        case Action::Jog :
                            machineData.state.mode = MachineState::Jog;
                            printf("Off -> Jog\n");
                            break;
                        case Action::Program :
                            machineData.state.mode = MachineState::PrepareMove;
                            printf("Off -> Program\n");
                            break;
                        default:
                            break;
                    }
                    break;

                case (static_cast<uint32_t>(MachineState::Jog)):
                    switch (instructionBuffer)
                    {
                        case Action::Cancel:
                            machineData.state.mode = MachineState::Off;
                            printf("Jog -> Off\n");
                            break;
                        default:
                            break;
                    }
                    break;
                    
                case (static_cast<uint32_t>(MachineState::PrepareMove)):
                    switch (instructionBuffer)
                    {
                        case Action::Data:
                            printf("Program: Data\n");
                            break;
                        case Action::Done:
                            machineData.state.mode = MachineState::WaitStart;
                            break;
                        case Action::Cancel:
                            machineData.state.mode = MachineState::Off;
                            printf("Program -> Off\n");
                            break;
                        default:
                            break;
                    }
                    break;

                case (static_cast<uint32_t>(MachineState::WaitStart)):
                    switch (instructionBuffer)
                    {
                        case Action::Cancel:
                            machineData.state.mode = MachineState::Off;
                            printf("Waitstart -> Off\n");
                            break;
                        case Action::Start:
                            machineData.state.mode = MachineState::ExecuteProgram;
                            printf("Waitstart -> Start\n");
                            break;
                        default:
                            break;
                    }
                    break;

                case (static_cast<uint32_t>(MachineState::ExecuteProgram)):
                    switch (instructionBuffer)
                    {
                        case Action::Next:
                            break;
                        case Action::Done:
                            machineData.state.mode = MachineState::Off;
                            break;
                        default:
                            break;
                    }
                    break;

                case (static_cast<uint32_t>(MachineState::EmergencyStop)):
                    switch (instructionBuffer)
                    {
                        case Action::Done:
                            machineData.state.mode = MachineState::Off;
                            machineData.state.emergencyStop = false;
                            printf("Emer -> Off\n");
                            break;
                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
            machineData.state.actionRequired = false;

            critical_section_exit(&stateManagerLock);
        }

        void setAction(Action instruction)
        {
            critical_section_enter_blocking(&stateManagerLock);
            instance->instructionBuffer = instruction;
            instance->machineData.state.actionRequired = true;
            critical_section_exit(&stateManagerLock);
        }

        void setEmergencyStop()
        {
            critical_section_enter_blocking(&stateManagerLock);
            instance->machineData.state.emergencyStop = true;
            instance->machineData.state.actionRequired = true;
            critical_section_exit(&stateManagerLock);
        }

};

StateManager* StateManager::stateManagerInstance{nullptr};


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
}



