/***********************************************************************
 * @file	:	data_reader.hpp
 * @brief 	:	Callbacks for welding-corner messages.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#include "data_reader.hpp"

namespace Communication {

    static StateManager *stateManager = StateManager::getInstance();

    void changeToJogCallback(void) {
        stateManager->setAction(Action::Jog);
    }

    void changeToProgramCallback(void) {
        stateManager->setAction(Action::Program);
    }

    void receiveTrajectoryDataCallback(void) {
        stateManager->setAction(Action::Data);
    }

    void cancelOperationCallback(void) {
        stateManager->setAction(Action::Cancel);
    }

    void startOperationCallback(void) {
        stateManager->setAction(Action::Start);
    }

    void emergencyStopCallback(void) {
        stateManager->setEmergencyStop();
    }

    void rxCallback(RxMessageId msgId, uint8_t dataLength, volatile uint8_t *msgData) {
        // If message ID exist in message map
        if (operationTable.count(msgId) == 1) {
            // If message length corresponds to operationTable length
            if (dataLength == operationTable.at(msgId).length) {
                // If message handler exists
                if (operationTable.at(msgId).rxMsgCallback != nullptr) {
                    // Call message handler
                    operationTable.at(msgId).rxMsgCallback();
                    stateManager->machineProcess();
                }
            }
        }
    }

    void txCallback(uint8_t *msgData) {
        *msgData = 0x07;
    }

} // namespace Communication
