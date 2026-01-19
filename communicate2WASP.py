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

payload = bytearray([])

payload.extend(short2bytes(40)) # extrude speed
payload.extend(short2bytes(0)) # extrude state


# do not change the following two values
payload.extend(short2bytes(200)) # ex2nozzleFactor
payload.extend(short2bytes(10000)) # chamber speed
payload.extend(short2bytes(1)) # chamber state
payload.extend(short2bytes(0)) # address
payload.extend(short2bytes(0)) # value

if __name__ == '__main__':
    publish.single(topic="wasp/state", payload=payload, hostname=host, auth={'username':"roboMQTT", 'password':"FIM4AMisgreat"})
