#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <cstdint>
#include "pico/stdlib.h"

#define i2c_baud_rate 100000U
#define i2c_slave_addr 0x55U

// Declare the bits in the registers we use
#define FIRST_DATA_BYTE (1 << 11)
#define DAT 0xFF
#define RD_REQ  (1 << 5)
#define RX_FULL   (1 << 2)

void i2c0_irq_handler();

typedef void (*RxHandler)(uint8_t data, bool isFirst);
typedef void (*TxHandler)(void);

class I2CSlave
{
    private:
        RxHandler rxHandler = nullptr;
        TxHandler txHandler = nullptr;

    public:
        I2CSlave(RxHandler rxHandler, TxHandler txHandler) 
            : rxHandler(rxHandler), txHandler(txHandler) 
        {
            this->init();
        }

        void init()
        {
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
            i2c0_hw->intr_mask | (RD_REQ | RX_FULL);

            // Install handler for the interruption
            irq_set_exclusive_handler(I2C0_IRQ, &i2c0_irq_handler);

            // Enable I2C0 interrupt
            irq_set_enabled(I2C0_IRQ, true);
        }

        // Interrupt handler
    void i2c0_irq_handler() 
    {
        // Get interrupt status
        uint32_t status = i2c0_hw->intr_stat;
        // Check if we have received data from the I2C master
        if (status & RX_FULL) {
            // Read the data from buffer (this will clear the interrupt)
            uint32_t value = i2c0_hw->data_cmd;
            bool isFirst = value & FIRST_DATA_BYTE;
            uint8_t data = value & DAT;
            
        }
        // Check to see if the I2C master is requesting data from us
        if (status & RD_REQ) {
            // Write the data to buffer
            i2c0_hw->data_cmd = (uint32_t) 0x07 & DAT;
            // Clear the interrupt
            i2c0_hw->clr_rd_req;
            printf("s %d\n", 0x07);
        }
    }

};


