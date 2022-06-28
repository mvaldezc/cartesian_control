#pragma once

class IDataHandler
{
    public:
        IDataHandler() = default;
        virtual ~IDataHandler() = default;
        virtual bool saveSerializedData(uint8_t dataLength, volatile uint8_t * msgData) = 0;
    };
