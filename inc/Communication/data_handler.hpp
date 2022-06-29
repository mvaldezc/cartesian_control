#pragma once
#include <cstdint>

template <typename P, typename T>
class DataHandler
{
    public:
        explicit DataHandler(P container): dataContainer(container){}

        bool saveSerializedData(const uint8_t dataLength, const volatile uint8_t * msgData)
        {
            // If data was received and message is not null
            if(dataLength > 0 && msgData != nullptr){
                for(int i = 0; i < dataLength; i++){
                    dataBuffer.bytes[i] = *(msgData+i);
                }
                dataContainer->push(dataBuffer.directData);
                return true;
            }
            return false;
        }

        void clearContainer()
        {
            long dataSize = dataContainer->size();
            for(int i = 0; i < dataSize; i++)
            {
                dataContainer->pop();
            }
        }
    private:

        union multiDataBuffer{
            uint8_t bytes[16];
            T directData;
        };

        multiDataBuffer dataBuffer = {.bytes = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

        P dataContainer = nullptr;
};
