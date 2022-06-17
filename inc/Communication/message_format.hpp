/***********************************************************************
 * @file	:	message_format.hpp
 * @brief 	:	Message Format Interface
 *              Signature of message frame handlers.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <cstdint>

namespace Communication {

    /**
     * @brief Message id data type 
     */
    typedef uint8_t RxMessageId;

    /**
     * @brief Function pointer signature that is called by the system when a received message
     *        interruption occurs.
     */
    typedef void (*RxHandler)(RxMessageId msgId, uint8_t dataLength, volatile uint8_t * data);

    /**
     * @brief Function pointer signature that is called by the system when a message request
     *        interruption occurs.
     */
    typedef void (*TxHandler)(uint8_t * data);

} // namespace Communication
