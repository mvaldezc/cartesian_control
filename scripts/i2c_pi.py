from smbus2 import SMBus
from time import sleep

pico_addr = 2 # bus address 
i2c = SMBus(1) # indicates /dev/i2c-1

numb = 8

print("Enter 1 for ON or 0 for OFF")
while True :
    i2c.write_byte(pico_addr,numb)
    b = i2c.read_byte(pico_addr)
    print(b)
    sleep(1)
