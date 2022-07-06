/***********************************************************************
 * @file	:	data_reader.hpp
 * @brief 	:	Callbacks for welding-corner messages.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>
#include <unordered_map>
#include <cstring>
#include "message_format.hpp"
#include "state_manager.hpp"
#include "trajectory_data.hpp"

namespace Communication {

    typedef void (*Callback)(const volatile uint8_t * msgData);

    constexpr RxMessageId CHANGE_TO_JOG = 0x00;
    constexpr RxMessageId CHANGE_TO_PROGRAM = 0x01;
    constexpr RxMessageId CHANGE_TO_LOAD = 0x02;
    constexpr RxMessageId DOWNLOAD_PROGRAM = 0x03;
    constexpr RxMessageId TRAJECTORY_DATA = 0x04;
    constexpr RxMessageId CANCEL_OPERATION = 0x05;
    constexpr RxMessageId START_OPERATION = 0x06;
    constexpr RxMessageId STOP_OPERATION = 0x07;
    constexpr RxMessageId EMERGENCY_STOP = 0x08;

    typedef struct 
    {
        uint8_t length;
        Callback rxMsgCallback;
    } RxMessageDataTemplate;

    void changeToJogCallback(const volatile uint8_t * msgData);

    void changeToProgramCallback(const volatile uint8_t * msgData);

    void changeToLoadCallback(const volatile uint8_t * msgData);

    void downloadProgramCallback(const volatile uint8_t * msgData);

    void receiveTrajectoryDataCallback(const volatile uint8_t * msgData);

    void cancelOperationCallback(const volatile uint8_t * msgData);

    void startOperationCallback(const volatile uint8_t * msgData);

    void stopOperationCallback(const volatile uint8_t * msgData);

    void emergencyStopCallback(const volatile uint8_t * msgData);

    const std::unordered_map<RxMessageId, RxMessageDataTemplate> messageDictionary =
    {
        {CHANGE_TO_JOG, {0, &changeToJogCallback}},
        {CHANGE_TO_PROGRAM, {0, &changeToProgramCallback}},
        {CHANGE_TO_LOAD, {0, &changeToLoadCallback}},
        {DOWNLOAD_PROGRAM, {1, &downloadProgramCallback}},
        {TRAJECTORY_DATA, {10, &receiveTrajectoryDataCallback}},
        {CANCEL_OPERATION, {0, &cancelOperationCallback}},
        {START_OPERATION, {0, &startOperationCallback}},
        {STOP_OPERATION, {0, &stopOperationCallback}},
        {EMERGENCY_STOP, {0, &emergencyStopCallback}}
    };

    void rxCallback(RxMessageId msgId, uint8_t dataLength, volatile uint8_t * msgData);

    void txCallback(uint8_t * msgData);

    void installDataContainer(path_list_t via_points);

} // namespace Communication
