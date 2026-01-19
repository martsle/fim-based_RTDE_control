#!/usr/bin/python3
import time
import serial

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")


serial_port = serial.Serial(
    port="COM3",
    #port="COM6",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
# Wait a second to let the port initialize

print("wait")
time.sleep(1)
print("go")

try:
    # Send a simple header
    
    while True:
        d = int(input("New delay:\n"))
        if d == 0:
            s = "{}\n".format(d).encode()
            print(s)
            serial_port.write(s)
            test = input("Quit program? (y/n)\n")
            if test.lower() == 'y':
                break
        if d < 200:
            print("WARNING, setting too fast!")
            continue

        s = "{}\n".format(d).encode()
        print(s)
        serial_port.write(s)
        
        time.sleep(0.1)

        while (serial_port.in_waiting):
            incoming = serial_port.readline()
            print(incoming)
        
        
    
    
    '''
    for n in range(1000):
        incoming = serial_port.readline()
        print(incoming)
        time.sleep(0.1)
    '''

except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass
