#pragma once
#include <cstdint>
#include <unordered_map>

enum class RxMessage : uint8_t
{
    CHANGE_TO_JOG,
    CHANGE_TO_PROGRAM,
    TRAJECTORY_DATA,
    CANCEL_OPERATION,
    START_OPERATION,
    EMERGENCY_STOP
}

typedef void (*Callback)(void);

typedef struct 
{
    uint8_t length;
    Callback rxCallback;
} RxMessageData;

std::unordered_map<RxMessage, RxMessageData> map =
    {
        {}}
