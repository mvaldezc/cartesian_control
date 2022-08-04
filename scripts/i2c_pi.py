from smbus2 import SMBus

pico_addr = 85 # bus address 
i2c = SMBus(1) # indicates /dev/i2c-1

msgids = {"CHANGE_TO_JOG" : 0, "CHANGE_TO_PROGRAM" : 1, "CHANGE_TO_LOAD" : 2, "DOWNLOAD_PROGRAM" : 3,
"TRAJECTORY_DATA" : 4, "CANCEL_OPERATION" : 5, "START_OPERATION" : 6, "STOP_OPERATION" : 7, "EMERGENCY_STOP" : 8}

def send_message(msgid, data = []):
    try:
        i2c.write_block_data(pico_addr,msgid, data)
        print("data sent, msgid: " + str(msgid) + " , dataSize: " + str(len(data)) + "\n")
    except Exception as e:
        print("write error\n")

def change_to_jog():
    send_message(msgids["CHANGE_TO_JOG"])

def change_to_program():
    send_message(msgids["CHANGE_TO_PROGRAM"])

def change_to_load():
    send_message(msgids["CHANGE_TO_LOAD"])

def download_program(programSize):
    assert(type(programSize) == int)
    send_message(msgids["DOWNLOAD_PROGRAM"], [programSize])

def send_trajectory_data(data):
    assert(type(data) == list)
    send_message(msgids["TRAJECTORY_DATA"], data)

def cancel_operation():
    send_message(msgids["CANCEL_OPERATION"])

def start_operation():
    send_message(msgids["START_OPERATION"])

def stop_operation():
    send_message(msgids["STOP_OPERATION"])

def emergency_stop():
    send_message(msgids["EMERGENCY_STOP"])


