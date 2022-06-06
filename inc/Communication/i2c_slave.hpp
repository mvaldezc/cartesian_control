#pragma once
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <cstdint>
#include <cstdio>
#include "pico/stdlib.h"
#include "message_format.hpp"

#define i2c_baud_rate 400000U
#define i2c_slave_addr 0x55U
#define BUFFER_SIZE 16U

// Declare the bits in the registers we use
#define FIRST_DATA_BYTE (1 << 11)
#define DAT 0xFF
#define RD_REQ  (1 << 5)
#define RX_FULL   (1 << 2)

namespace Communication {
    class I2CSlave
    {
        public:
            
            static void init(RxHandler rxHandlerPtr, TxHandler txHandlerPtr)
            {
                rxHandler = rxHandlerPtr;
                txHandler = txHandlerPtr;

                #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
                #error i2c software component requires a board with I2C pins
                #endif

                // Use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
                i2c_init(i2c_default, i2c_baud_rate);
                gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
                gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
                gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
                gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
                i2c_set_slave_mode(i2c0, true, i2c_slave_addr);

                // Set interrupt mask of i2c0 hardware (activate slave write and slave read)
                i2c0_hw->intr_mask = (RD_REQ | RX_FULL);

                // Install handler for the interruption
                irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);

                // Enable I2C0 interrupt
                irq_set_enabled(I2C0_IRQ, true);
            }

        private:
            I2CSlave(){}
            static volatile RxHandler rxHandler;
            static volatile TxHandler txHandler;
            static volatile uint8_t rxBuffer[BUFFER_SIZE];
            static volatile uint8_t receivedDataLength;
            
            static void i2c0_irq_handler()
            {
                // Get interrupt status
                uint32_t status = i2c0_hw->intr_stat;
                // Check if we have received data from the I2C master
                if (status & RX_FULL) {
                    rx_irq();
                }

                // Check to see if the I2C master is requesting data from us
                if (status & RD_REQ) {
                    tx_irq();
                }
            }

            static void rx_irq()
            {
                // Read the data from FIO I2C buffer (this will clear the interrupt)
                uint32_t value = i2c0_hw->data_cmd;
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
                i2c0_hw->data_cmd = (uint32_t) (data & DAT);
                // Clear the interruption
                i2c0_hw->clr_rd_req;
            }

            static inline void cleanBuffer(volatile uint8_t * buffer)
            {
                for(int i = 0; i < BUFFER_SIZE; i++)
                {
                    *(buffer+i) = 0;
                }
            }
    };
}
