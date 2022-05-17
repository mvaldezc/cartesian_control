#pragma once
#include <cstdint>
#include <unordered_map>
#include "state_manager.hpp"

typedef void (*Callback)(void);

typedef uint8_t RxMessageId;

constexpr RxMessageId CHANGE_TO_JOG = 0x00;
constexpr RxMessageId CHANGE_TO_PROGRAM = 0x01;
constexpr RxMessageId TRAJECTORY_DATA = 0x02;
constexpr RxMessageId CANCEL_OPERATION = 0x03;
constexpr RxMessageId START_OPERATION = 0x04;
constexpr RxMessageId EMERGENCY_STOP = 0x05;

typedef struct 
{
    uint8_t length;
    Callback rxCallback;
} RxMessageData;

void changeToJogCallback(void)
{
    instructionBuffer = Action::Jog;
}

void changeToProgramCallback(void)
{
    instructionBuffer = Action::Program;
}

void receiveTrajectoryDataCallback(void)
{
    instructionBuffer = Action::Data;
}

void cancelOperationCallback(void)
{
    instructionBuffer = Action::Cancel;
}

void startOperationCallback(void)
{
    instructionBuffer = Action::Start;
}

void emergencyStopCallback(void)
{
    machineData.state.emergencyStop = true;
}

std::unordered_map<RxMessageId, RxMessageData> operationMap =
{
    {CHANGE_TO_JOG, {1, &changeToJogCallback}},
    {CHANGE_TO_PROGRAM, {1, &changeToProgramCallback}},
    {TRAJECTORY_DATA, {17, &receiveTrajectoryDataCallback}},
    {CANCEL_OPERATION, {1, &cancelOperationCallback}},
    {START_OPERATION, {1, &startOperationCallback}},
    {EMERGENCY_STOP, {1, &emergencyStopCallback}}
};

void rxCallback(RxMessageId msgId, uint8_t * msgData, uint8_t dataLength)
{
    if(operationMap.count(msgId) == 1)
    {
        if(dataLength == operationMap.at(msgId).length)
        {
            if(operationMap.at(msgId).rxCallback != NULL)
            {
                operationMap.at(msgId).rxCallback();
                machineData.state.actionNotRequired = false;
            }
        }
    }
}
