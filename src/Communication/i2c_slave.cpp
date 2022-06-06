#include "i2c_slave.hpp"

namespace Communication {
    volatile RxHandler I2CSlave::rxHandler = nullptr;
    volatile TxHandler I2CSlave::txHandler = nullptr;
    volatile uint8_t I2CSlave::rxBuffer[BUFFER_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    volatile uint8_t I2CSlave::receivedDataLength = 0;
}
