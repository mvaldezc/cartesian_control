/***********************************************************************
 * @file	:	data_reader.hpp
 * @brief 	:	Callbacks for welding-corner messages.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#include "communication_handler.hpp"

namespace Communication {

    static uint8_t programSize = 0;
    static uint8_t dataCounter = 0;
    using Algorithm::TrajectoryGeneration::path_params_t;
    static std::unique_ptr<DataHandler<path_list_t, path_params_t>> trajectoryDataHandler;
    static StateManager *stateManager = StateManager::getInstance();

    void changeToJogCallback(const volatile uint8_t * msgData) {
        stateManager->setAction(Action::Jog);
    }

    void changeToProgramCallback(const volatile uint8_t * msgData) {
        stateManager->setAction(Action::Program);
    }

    void changeToLoadCallback(const volatile uint8_t * msgData) {
        stateManager->setAction(Action::Load);
    }

    void downloadProgramCallback(const volatile uint8_t * msgData) {
        if(stateManager->getMachineState() == MachineState::LoadProgram)
        {
            if(trajectoryDataHandler)
            {
                trajectoryDataHandler->clearContainer();
                programSize = * msgData;
                dataCounter = 0;
            }
        }
    }

    void receiveTrajectoryDataCallback(const volatile uint8_t * msgData) {
        if(stateManager->getMachineState() == MachineState::LoadProgram)
        {
            if(trajectoryDataHandler && dataCounter < programSize)
            {
                trajectoryDataHandler->saveSerializedData(messageDictionary.at(TRAJECTORY_DATA).length, msgData);
                if(++dataCounter == programSize)
                {
                    stateManager->setAction(Action::Done);
                }
            }
        }
    }

    void cancelOperationCallback(const volatile uint8_t * msgData) {
        stateManager->setAction(Action::Cancel);
    }

    void startOperationCallback(const volatile uint8_t * msgData) {
        stateManager->setAction(Action::Start);
    }

    void emergencyStopCallback(const volatile uint8_t * msgData) {
        stateManager->setEmergencyStop();
    }

    void rxCallback(RxMessageId msgId, uint8_t dataLength, volatile uint8_t * msgData) 
    {
        // If message ID exist in messageDictionary
        if (messageDictionary.count(msgId) == 1) {
            // If message length corresponds to messageDictionary length
            if (dataLength == messageDictionary.at(msgId).length) {
                // If message handler exists
                if (messageDictionary.at(msgId).rxMsgCallback != nullptr) {
                    // Call message handler
                    messageDictionary.at(msgId).rxMsgCallback(msgData);
                    stateManager->machineProcess();
                }
            }
        }
    }

    void txCallback(uint8_t * msgData) {
        *msgData = 0x07;
    }

    void installDataContainer(path_list_t dataContainer) {
        trajectoryDataHandler = std::make_unique<DataHandler<path_list_t, path_params_t>>(dataContainer);
    }

} // namespace Communication
