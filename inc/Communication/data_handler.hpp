#pragma once
#include "idata_handler.hpp"

template <typename P, typename T>
class DataHandler : public IDataHandler
{
    public:
        DataHandler(P container): dataContainer(container){}

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
