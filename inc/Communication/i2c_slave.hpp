/***********************************************************************
 * @file	:	i2c_slave.hpp
 * @brief 	:	I2C Slave Library
 * 				Library that enables i2c slave functionality and manages
 *              messages.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once

#include <cstdint>
#include <cstdio>
#include "message_format.hpp"

#ifdef RASP_PICO

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

// Declare the bits in the registers we use
#define FIRST_DATA_BYTE (1 << 11)
#define DAT 0xFF
#define RD_REQ (1 << 5)
#define RX_FULL (1 << 2)
#define RX_IRQ RX_FULL
#define TX_IRQ RD_REQ
// Declare registers
#define I2C_INTR_STATUS (i2c0_hw->intr_stat)
#define CLEAR_I2C_TX_INT (i2c0_hw->clr_rd_req)
#define DATA_BUFFER (i2c0_hw->data_cmd)

#endif

#define I2C_BAUD_RATE 400000U // I2C baud rate
#define I2C_SLAVE_ADDRESS 0x55U // I2C device address
#define BUFFER_SIZE 16U // Maximum message lenght in bytes

namespace Communication {

    /**
     * @class I2CSlave
     * @brief Static class that configures I2C hardware as a slave and 
     * installs callbacks for read and write ISRs.
     */
    class I2CSlave
    {
        public:
            
            /**
             * @brief Static method that initializes I2C hardware as slave.
             * @param[in] rxHandlerPtr Pointer to function that will be called
             *            once a message is received.
             * @param[in] txHandlerPtr Pointer to function that will be called
             *            once a message request is received.
             */
            static void init(RxHandler rxHandlerPtr, TxHandler txHandlerPtr)
            {
                rxHandler = rxHandlerPtr;
                txHandler = txHandlerPtr;

                #ifdef RASP_PICO
                // Use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
                i2c_init(i2c_default, I2C_BAUD_RATE);
                gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
                gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
                gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
                gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
                i2c_set_slave_mode(i2c0, true, I2C_SLAVE_ADDRESS);

                // Set interrupt mask of i2c0 hardware (activate slave write and slave read)
                i2c0_hw->intr_mask = (RD_REQ | RX_FULL);

                // Install handler for the interruption
                irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);

                // Enable I2C0 interrupt
                irq_set_enabled(I2C0_IRQ, true);
                #endif
            }

        private:
            I2CSlave(){} // Private constructor to avoid instance creation
            static volatile RxHandler rxHandler;
            static volatile TxHandler txHandler;
            static volatile uint8_t rxBuffer[BUFFER_SIZE];
            static volatile uint8_t receivedDataLength;
            
            static void i2c0_irq_handler()
            {
                // Get interrupt status
                uint32_t status = I2C_INTR_STATUS;
                // Check if we have received data from the I2C master
                if (status & RX_IRQ) {
                    rx_irq();
                }

                // Check to see if the I2C master is requesting data from us
                if (status & TX_IRQ) {
                    tx_irq();
                }
            }

            static void rx_irq()
            {
                // Read the data from FIFO I2C buffer (this will clear the interrupt)
                uint32_t value = DATA_BUFFER;
                bool isFirst = value & FIRST_DATA_BYTE;
                uint8_t data = value & DAT;
                printf("%d ", data);

                if(isFirst)
                {
                    rxBuffer[0]= data;
                    receivedDataLength = 1;
                }
                else if(receivedDataLength > 0)
                {   
                    rxBuffer[receivedDataLength] = data;
                    receivedDataLength++;

                    // If expected amount of data is received
                    if(rxBuffer[1] + 2 == receivedDataLength)
                    {
                        if(rxHandler != nullptr){
                            rxHandler(rxBuffer[0], rxBuffer[1], &rxBuffer[2]);
                        }
                        cleanBuffer(rxBuffer);
                        receivedDataLength = 0;
                        printf("\n");
                    }
                }
                // else: extra bytes are withdrawn 
            }

            static void tx_irq()
            {
                uint8_t data;
                if(txHandler != nullptr)
                {
                    txHandler(&data);
                }
                // Write the data to buffer
                DATA_BUFFER = (uint32_t) (data & DAT);
                // Clear the interruption
                CLEAR_I2C_TX_INT;
            }

            static inline void cleanBuffer(volatile uint8_t * buffer)
            {
                for(int i = 0; i < BUFFER_SIZE; i++)
                {
                    *(buffer+i) = 0;
                }
            }
    };
    
} // namespace Communication
