# from pyModbusTCP.client import ModbusClient
# import time

# c = ModbusClient(host="10.152.213.20", port=502, timeout=1.5, auto_open=True, auto_close=True)
# extrudeSpeed = 20
# extrudeState = 0
# ex2nozzleFactor = 200
# chamberSpeed = 15000
# chamberState = 0
# address = 0
# value = 0

# if c.write_multiple_registers(0, [extrudeSpeed, extrudeState, ex2nozzleFactor, chamberSpeed, chamberState]):
#     print("write ok")
#     errMsgs = c.read_holding_registers(0, 11)
#     print(errMsgs)
# else:
#     print("write error")


import paho.mqtt.publish as publish
import struct

host = "10.152.213.31"
short2bytes = struct.Struct('>H').pack

# state values: 
# nozzle motor speed (RPM, 1-65535) - 16 bit, 
# state inferred from speed
stateMsg = bytearray([0x00, 0x00])
# config values: 
# extruderMode (0: open loop, 1-25000: closed loop target current [0.01 A]) - 16 bit, 
# chamberSpeed (RPM) - 16 bit, 
# chamberState inferred from chamberSpeed,, 
# extruder2nozzleFactor (%, 100-500) - 16 bit, 
# driverAddress - 16 bit, 
# driverValue - 16 bit
configMsg = bytearray([0x00, 0x00, 0x27, 0x10, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00])

# state msg
stateMsg[0:2] = short2bytes(0) # extrude speed

# do not change the following two values
# config msg
configMsg[0:2] = short2bytes(0) # extruder mode
configMsg[2:4] = short2bytes(10000) # ex2nozzleFactor
configMsg[4:6] = short2bytes(200) # chamber speed
configMsg[6:8] = short2bytes(0) # address
configMsg[8:10] = short2bytes(0) # value

if __name__ == '__main__':
    publish.single(topic="wasp/state", payload=stateMsg, hostname=host, auth={'username':"roboMQTT", 'password':"FIM4AMisgreat"})
    publish.single(topic="wasp/config", payload=configMsg, hostname=host, auth={'username':"roboMQTT", 'password':"FIM4AMisgreat"})