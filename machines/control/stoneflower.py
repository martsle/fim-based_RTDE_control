import serial
import logging
import numpy as np
from machines.control.extruderController import ExtruderControl

logger = logging.getLogger(__name__)

class Stoneflower3D(ExtruderControl):
    def __init__(self, port:str = "/dev/ttyUSB0") -> None:
        super().__init__(host=port, gearRatio=46.66)        
        self._prescaler: int = 8
        self._fArduino: int  = 16000000 # [Hz]
        self._microstepping: int = 8
        self._motorSPR: int = 200 # [steps per revolution]
        self._pitch: float = 0.003 # m/revolution
        self._dBarrel: float = 0.0678 # m

        self._serial_port: serial.Serial = None

    @property
    def tps(self) -> float:
        """
        Tiks per second (Arduino interrupt).
        """
        return self._fArduino / self._prescaler
    
    @property
    def spr(self) -> float:
        """
        Steps for one extruder revolution.
        """
        return self._motorSPR * self._microstepping * self._gearRatio

    @property
    def barrelCS(self) -> float:
        """
        Barrel cross-section.
        """
        return self._dBarrel**2 / 4 * np.pi

    @property
    def vps(self) -> float:
        """
        Volume per ustep.
        """
        return self._pitch / self.spr * self.barrelCS

    def connect(self) -> None:
        try:
            self._serial_port = serial.Serial(
                port=self._host, 
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
        except:
            self._serial_port = None
            logging.warning(f"Serial connection not found.")

    def extrude(self, velMagnitude:float, hLayer:float) -> None:
        if velMagnitude <= 0:
            s = "{}\n".format(int(0)).encode()
            self._serial_port.write(s)
            logging.info(f"Extruder speed = 0 | Sending {s}")
            return 

        Q = self._filamentWidth * self._calibrationFactor * hLayer * velMagnitude # assuming velMagnitude [m/sec]

        extruder_s = self.vps / Q * self.tps
        if isinstance(self._serial_port, type(None)):
            logging.info(f"Extruder speed = {extruder_s} ms | Not connected -> not sending")
            return

        if extruder_s > 65000:
            s = "{}\n".format(0).encode()
            self._serial_port.write(s)
            logging.info(f"Extruder speed too slow sending 0")
        elif extruder_s > 200:
            s = "{}\n".format(int(extruder_s)).encode()
            self._serial_port.write(s)
            logging.info(f"Extruder speed = {extruder_s} ms | Sending {s}")
            logging.info(f"robot speed = {velMagnitude} | vps = {self.vps}")
        else:   
            s = "{}\n".format(int(200)).encode()
            self._serial_port.write(s)
            logging.info(f"Extruder running at maximum speed: Delay adjusted to 200ms | Sending {s} ")
    
    def stop(self):
        if isinstance(self._serial_port, type(None)):
            logging.info(f"Not connected -> not sending")
            return
        s = "{}\n".format(65534).encode()
        self._serial_port.write(s)
        logging.info(f"Extruder stopped")