import logging
import numpy as np
import paho.mqtt.publish as publish
import struct
from machines.control.extruderController import ExtruderControl

logger = logging.getLogger(__name__)
short2bytes = struct.Struct('>H').pack

class WASPclay(ExtruderControl):
    def __init__(self, host: str,  port: int = 1883, timeout: float = 1.0, gearRatio: float = 20.0, auth: dict = None) -> None:
        super().__init__(host=host, port=port, gearRatio=gearRatio)
        self._timeout: float = timeout
        self._pitch:float = 0.0135 # m/revolution
        self._dBarrel:float = 0.024 # m
        self._dScrewAxis:float = 0.009 # m
        self._auth = {'username': "", 'password': ""}
        if auth is not None:
            self._auth.update(auth)

        # state values: 
        # nozzle motor speed (RPM, 1-65535) - 16 bit, 
        # state inferred from speed
        self._nozzleSpeed = bytearray([0x00, 0x00])
        # config values: 
        # extruderMode (0: open loop, 1-25000: closed loop target current [0.01 A]) - 16 bit, 
        # chamberSpeed (RPM) - 16 bit, 
        # chamberState inferred from chamberSpeed,, 
        # extruder2nozzleFactor (%, 100-500) - 16 bit, 
        # driverAddress - 16 bit, 
        # driverValue - 16 bit
        self._waspConfig = bytearray([0x00, 0x00, 0x27, 0x10, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00])

    @property
    def vpr(self) -> float:
        """
        Effective volume per revolution.
        """
        return (self._dBarrel**2 - self._dScrewAxis**2) / 4 * np.pi * self._pitch

    def setAuthentication(self, username: str, password: str) -> None:
        self._auth['username'] = username
        self._auth['password'] = password

    def changeConfiguration(self, **kvpars) -> None:
        for key, value in kvpars.items():
            if key == 'extruderMode':
                self.changeMode(value)
            elif key == 'chamberSpeed':
                self.changeChamberSpeed(value)
            elif key == 'e2nFactor':
                self.changeExtruder2NozzleFactor(value)
            elif key == 'driverParameter':
                self.changeDriverParameter(value, kvpars.get('driverValue', 0))
        publish.single(topic="wasp/config", payload=self._waspConfig, hostname=self._host, auth=self._auth)
        self.changeDriverParameter()

    def changeMode(self, extruderMode: int = 0) -> None:
        self._waspConfig[0:2]  = short2bytes(extruderMode & 0xFFFF)

    def changeChamberSpeed(self, chamberSpeed: int = 10000) -> None:
        self._waspConfig[2:4]  = short2bytes(chamberSpeed & 0xFFFF)
    
    def changeExtruder2NozzleFactor(self, extruder2nozzleFactor: int = 200) -> None:
        self._waspConfig[4:6]  = short2bytes(extruder2nozzleFactor & 0xFFFF)

    def changeDriverParameter(self, driverAddress: int = 0, driverValue: int = 0) -> None:
        self._waspConfig[6:8]  = short2bytes(driverAddress & 0xFFFF)
        self._waspConfig[8:10] = short2bytes(driverValue & 0xFFFF)

    def extrude(self, velMagnitude:float, hLayer:float) -> None:
        if velMagnitude <= 0:
            logging.info(f"Extruder speed = 0 | Sending nothing")
            return 

        Q = self._filamentWidth * hLayer * velMagnitude * self._calibrationFactor # assuming velMagnitude [m/sec]
        extruder_s = int(Q / self.vpr * 60 * self._gearRatio) # nozzle speed in RPM

        if extruder_s > 100 * self._gearRatio: # max extruder speed
            self._nozzleSpeed = bytearray([0x00, 0x00]) # nozzle motor speed
            logging.info(f"Motor speed too fast. Emergency stop...")
        elif extruder_s > 0:
            self._nozzleSpeed[0:2] = short2bytes(extruder_s & 0xFFFF) # nozzle motor speed
            logging.info(f"Sending nozzle motor speed = {extruder_s} RPM")
            logging.info(f"robot speed = {velMagnitude} m/s | layer height = {hLayer} m")
        else:   
            self._nozzleSpeed = bytearray([0x00, 0x00]) # nozzle motor speed
            logging.info(f"Impossible speed: Emergency stop...")

        publish.single(topic="wasp/state", payload=self._nozzleSpeed, hostname=self._host, auth=self._auth)

    def stop(self):
        self._nozzleSpeed = bytearray([0x00, 0x00]) # nozzle motor speed
        publish.single(topic="wasp/state", payload=self._nozzleSpeed, hostname=self._host, auth=self._auth)
        logging.info(f"Extruder stopped")