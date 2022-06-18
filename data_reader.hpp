/***********************************************************************
 * @file	:	data_reader.hpp
 * @brief 	:	Callbacks for welding-corner messages.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>
#include <unordered_map>
#include "message_format.hpp"
#include "state_manager.hpp"

namespace Communication {

    typedef void (*Callback)(void);

    constexpr RxMessageId CHANGE_TO_JOG = 0x00;
    constexpr RxMessageId CHANGE_TO_PROGRAM = 0x01;
    constexpr RxMessageId TRAJECTORY_DATA = 0x02;
    constexpr RxMessageId CANCEL_OPERATION = 0x03;
    constexpr RxMessageId START_OPERATION = 0x04;
    constexpr RxMessageId EMERGENCY_STOP = 0x05;

    typedef struct 
    {
        uint8_t length;
        Callback rxMsgCallback;
    } RxMessageDataTemplate;

    static StateManager * stateManager = StateManager::getInstance();

    void changeToJogCallback(void)
    {
        stateManager->setAction(Action::Jog);
    }

    void changeToProgramCallback(void)
    {
        stateManager->setAction(Action::Program);
    }

    void receiveTrajectoryDataCallback(void)
    {
        stateManager->setAction(Action::Data);
    }

    void cancelOperationCallback(void)
    {
        stateManager->setAction(Action::Cancel);
    }

    void startOperationCallback(void)
    {
        stateManager->setAction(Action::Start);
    }

    void emergencyStopCallback(void)
    {
        stateManager->setEmergencyStop();
    }

    std::unordered_map<RxMessageId, RxMessageDataTemplate> operationTable =
    {
        {CHANGE_TO_JOG, {0, &changeToJogCallback}},
        {CHANGE_TO_PROGRAM, {0, &changeToProgramCallback}},
        {TRAJECTORY_DATA, {10, &receiveTrajectoryDataCallback}},
        {CANCEL_OPERATION, {0, &cancelOperationCallback}},
        {START_OPERATION, {0, &startOperationCallback}},
        {EMERGENCY_STOP, {0, &emergencyStopCallback}}
    };

    void rxCallback(RxMessageId msgId, uint8_t dataLength, volatile uint8_t * msgData)
    {
        // If message ID exist in message map
        if(operationTable.count(msgId) == 1)
        {
            // If message length corresponds to operationTable length
            if(dataLength == operationTable.at(msgId).length)
            {
                // If message handler exists
                if(operationTable.at(msgId).rxMsgCallback != nullptr)
                {
                    // Call message handler
                    operationTable.at(msgId).rxMsgCallback();
                    machineProcess();
                }
            }
        }
    }

    void txCallback(uint8_t * msgData)
    {
        *msgData = 0x07;
    }

}
