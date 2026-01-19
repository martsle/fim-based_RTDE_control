from machines.connection.kukaconnect import KUKAconnectionKRC4
import time

IP = "localhost"             # TBD
#IP = "10.152.49.123"             # TBD

PORT = 7000
var = "PING" #The variable of structure type E6AXIS contains the current axis angles or axis positions.

kUKA = KUKAconnectionKRC4(IP, PORT)
'''
val = KUKA.read(var)
print("##### read test #####")
print(f"The Struct {var} contains the following values: {val}")
print(f"{var} is a {type(val)}")
print(f"if {var} consits of differen types:")
for x in val:
    print(f"-- {type(x)}")
'''
print("##### write test #####")
#test = kUKA.discoverRobots()
#test = kUKA.writeVar(var, str(1), False)

times = []
msgs = []
start = time.process_time()
for n in range(10):
    test = kUKA.readVar(var)
    end = time.process_time()
    times.append(end-start)
    msgs.append(test)
for t, m in zip(times,msgs):
    print(f"Reply message at {t:10.10f}: {m}")
#kUKA.disconnect()