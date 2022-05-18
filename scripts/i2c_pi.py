from smbus2 import SMBus
from time import sleep

pico_addr = 85 # bus address 
i2c = SMBus(1) # indicates /dev/i2c-1

numb = 6

print("Enter 1 for ON or 0 for OFF")
while True :
    
    try:
        i2c.write_byte(pico_addr,numb)
        print("data sent")
    except Exception as e:
        print("write error")
        continue
    
    sleep(2)
    
    read = 0
    while read == 0:
        try:
            msg = i2c.read_byte(pico_addr)
            print("data received : " + str(msg) )
            read = 1
        except Exception as e:
            print("read error")
            continue
        sleep(1)
    
    print("done")
    
