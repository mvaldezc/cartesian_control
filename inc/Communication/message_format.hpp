/***********************************************************************
 * @file	:	message_format.hpp
 * @brief 	:	Template of messages frames
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>

namespace Communication {
    typedef uint8_t RxMessageId;

    typedef void (*RxHandler)(RxMessageId msgId, uint8_t dataLength, volatile uint8_t * data);
    typedef void (*TxHandler)(uint8_t * data);
}

