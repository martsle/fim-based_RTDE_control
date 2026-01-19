class ExtruderControl:
    def __init__(self, host: str = "10.152.213.31",  port: int = 1883, gearRatio=20.0) -> None:
        self._host: str = host
        self._port: int = port
        self._filamentWidth = 0
        self._calibrationFactor:float = 2.5
        self._gearRatio:float = gearRatio
    
    def setFilamentWidth(self, width):
        self._filamentWidth = width
    
    def setCalibrationF(self, factor):
        self._calibrationFactor = factor
    
    def connect(self) -> None:
        pass

    def extrude(self, velMagnitude:float, hLayer:float) -> None:
        pass

    def stop(self):
        pass

    def disconnect(self) -> None:
        pass
