from typing import List
from smbus2 import SMBus
from time import sleep

pico_addr = 85 # bus address 
i2c = SMBus(1) # indicates /dev/i2c-1

numb = 256

empty = []
data = [20, 21, 22, 23]

print("Enter 1 for ON or 0 for OFF")
while True :
    
    try:
        #i2c.write_byte(pico_addr,numb)
        i2c.write_block_data(pico_addr,0, empty)
        print("data sent")
    except Exception as e:
        print("write error")
        sleep(2)
        continue
    
    sleep(1)

    try:
        #i2c.write_byte(pico_addr,numb)
        i2c.write_block_data(pico_addr,1, empty)
        print("data sent")
    except Exception as e:
        print("write error")
        sleep(2)
        continue

    try:
        #i2c.write_byte(pico_addr,numb)
        i2c.write_block_data(pico_addr,2, data)
        print("data sent")
    except Exception as e:
        print("write error")
        sleep(2)
        continue

    try:
        #i2c.write_byte(pico_addr,numb)
        i2c.write_block_data(pico_addr,3, empty)
        print("data sent")
    except Exception as e:
        print("write error")
        sleep(2)
        continue

    try:
        #i2c.write_byte(pico_addr,numb)
        i2c.write_block_data(pico_addr,4, empty)
        print("data sent")
    except Exception as e:
        print("write error")
        sleep(2)
        continue

    try:
        #i2c.write_byte(pico_addr,numb)
        i2c.write_block_data(pico_addr,5, empty)
        print("data sent")
    except Exception as e:
        print("write error")
        sleep(2)
        continue
    
    read = 0
    while read == 0:
        try:
            msg = i2c.read_byte(pico_addr)
            print("data received : " + str(msg) )
            read = 1
        except Exception as e:
            print("read error")
            sleep(2)
            continue
        sleep(3)
    
    sleep(2)
    
