import i2c_pi
from time import sleep

programSize = 1
programData = [37, 32, 78, 0, 160, 15, 0, 0, 0, 0]

x = input("Enter a key to start")

i2c_pi.change_to_load()
i2c_pi.download_program(programSize)
i2c_pi.send_trajectory_data(programData)
i2c_pi.start_operation()
sleep(2)
#i2c_pi.emergency_stop()
i2c_pi.stop_operation()
sleep(2)
#i2c_pi.emergency_stop()
i2c_pi.start_operation()
