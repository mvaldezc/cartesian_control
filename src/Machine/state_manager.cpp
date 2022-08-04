#include "state_manager.hpp"

void StateManager::setAction(Action && instruction)
{
    critical_section_enter_blocking(&stateManagerLock);
    instance->instructionBuffer = instruction;
    instance->machineData.state.actionNotRequired = false;
    critical_section_exit(&stateManagerLock);
}

void StateManager::setEmergencyStop()
{
    critical_section_enter_blocking(&stateManagerLock);
    instance->machineData.state.emergencyStop = true;
    instance->machineData.state.actionNotRequired = false;
    critical_section_exit(&stateManagerLock);
}

MachineState StateManager::getMachineState()
{
    critical_section_enter_blocking(&stateManagerLock);
    MachineState returnval = machineData.state.mode;
    critical_section_exit(&stateManagerLock);
    return returnval;
}

void StateManager::machineProcess()
{
    critical_section_enter_blocking(&stateManagerLock);

    // Only if action needed, do something
    // Only if emergency is enabled, ignore everything else
    switch (machineData.key)
    {
        case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::Off)):
        case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::LoadProgram)):
        case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::WaitStart)):
        case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::ExecuteProgram)):
        case (EMERGENCY_ENABLED | static_cast<uint32_t>(MachineState::ProgramStop)):
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
                case Action::Load :
                    programLoaded = false;
                    machineData.state.mode = MachineState::LoadProgram;
                    printf("Off -> LoadProgram\n");
                    break;
                case Action::Program :
                    if(programLoaded)
                    {
                        machineData.state.mode = MachineState::WaitStart;
                        printf("Off -> WaitStart\n");
                    }
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
            
        case (static_cast<uint32_t>(MachineState::LoadProgram)):
            switch (instructionBuffer)
            {
                case Action::Done:
                    programLoaded = true;
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
                case Action::Stop:
                    machineData.state.mode = MachineState::ProgramStop;
                    break;
                case Action::Done:
                    machineData.state.mode = MachineState::Off;
                    break;
                default:
                    break;
            }
            break;

        case (static_cast<uint32_t>(MachineState::ProgramStop)):
            switch (instructionBuffer)
            {
                case Action::Start:
                    machineData.state.mode = MachineState::ExecuteProgram;
                    break;
                case Action::Cancel:
                    machineData.state.mode = MachineState::Off;
                default:
                    break;
            }
            break;

        case (static_cast<uint32_t>(MachineState::EmergencyStop)):
            // Needs to be restarted
            break;

        default:
            break;
    }
    machineData.state.actionNotRequired = true;
    instance->instructionBuffer = Action::None;

    critical_section_exit(&stateManagerLock);
}

StateManager * StateManager::instance{nullptr};
