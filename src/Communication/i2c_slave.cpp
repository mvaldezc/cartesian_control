#include "i2c_slave.hpp"

namespace Communication {
    RxHandler I2CSlave::rxHandler = nullptr;
    TxHandler I2CSlave::txHandler = nullptr;
    volatile uint8_t I2CSlave::rxBuffer[I2C_BUFFER_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0};
    volatile uint8_t I2CSlave::receivedDataLength = 0;
} // namespace Communication
