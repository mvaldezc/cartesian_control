/***********************************************************************
 * @file	:	data_reader.hpp
 * @brief 	:	Callbacks for welding-corner messages.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>
#include <unordered_map>
#include <queue>
#include <list>
#include <memory>
#include "message_format.hpp"
#include "state_manager.hpp"
#include "trajectory_data.hpp"

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

    void changeToJogCallback(void);

    void changeToProgramCallback(void);

    void receiveTrajectoryDataCallback(void);

    void cancelOperationCallback(void);

    void startOperationCallback(void);

    void emergencyStopCallback(void);

    const std::unordered_map<RxMessageId, RxMessageDataTemplate> messageDictionary =
    {
        {CHANGE_TO_JOG, {0, &changeToJogCallback}},
        {CHANGE_TO_PROGRAM, {0, &changeToProgramCallback}},
        {TRAJECTORY_DATA, {10, &receiveTrajectoryDataCallback}},
        {CANCEL_OPERATION, {0, &cancelOperationCallback}},
        {START_OPERATION, {0, &startOperationCallback}},
        {EMERGENCY_STOP, {0, &emergencyStopCallback}}
    };

    void rxCallback(RxMessageId msgId, uint8_t dataLength, volatile uint8_t * msgData);

    void txCallback(uint8_t * msgData);

    using path_data = ::Algorithm::TrajectoryGeneration::path_params_t;
    using path_struct_ptr = std::shared_ptr<std::queue<path_data, std::list<path_data>>>;
    
    template <typename P, typename T>
    class DataManager
    {
        public:
            DataManager(P container): dataContainer(container){}

            bool saveSerializedData(uint8_t dataLength, volatile uint8_t * msgData)
            {                
                // If data was received and message is not null
                if(dataLength > 0 && dataLength <=16 && msgData != nullptr){
                    for(int i = 0; i < dataLength; i++){
                        dataBuffer.bytes[i] = *(msgData+i);
                    }
                    dataContainer->push(dataBuffer.directData);
                    return true;
                }
                return false;
            }
        private:

            union multiDataBuffer{
                uint8_t bytes[16];
                T directData;
            };

            multiDataBuffer dataBuffer = {.bytes = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

            P dataContainer = nullptr;
    };

    void initDataManager(DataManager<path_struct_ptr, path_data> * dataManager);

} // namespace Communication
