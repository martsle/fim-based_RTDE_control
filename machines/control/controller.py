import numpy as np
import logging
from abc import abstractmethod
from typing import Optional, TYPE_CHECKING
from machines.connection import *
from machines.machinetypes import ToolType
from machines.control.stoneflower import Stoneflower3D
from machines.control.waspclay import WASPclay
from dataconnections.fimDataHandler import PathData

if TYPE_CHECKING:
    from machines.control.extruderController import ExtruderControl
    
logger = logging.getLogger(__name__)

class Controller:
    def __init__(self, robotConnection: RobotConnection, extruder: 'ExtruderControl') -> None:
        self._pathData: PathData = None
        self._extruder = extruder
        self._connection = robotConnection
        self.jointHistory = np.empty((0,6))
        self.TCPHistory = np.empty((0,6))
        self._debug: bool = False
        self._actualSpeed = np.zeros((1,6))
        self._currentState: list = None
        self._study = 'TEST'
        self._exp = 'TEST_1'

        self.ptCounter = 0
        self.lastState = 0
        self.lastRegis = 10
    
    def setPath(self, pd: PathData):
        self._pathData = pd
    
    def setParameters(self, filamentWidth=0.006, calibrationFactor=2.5, study = 'TEST', exp = 'TEST_1', **kvargs):
        self.setFilamentWidth(filamentWidth)
        self.setCalibrationF(calibrationFactor)
        self._study = study
        self._exp = exp

    def setFilamentWidth(self, width):
        self._extruder.setFilamentWidth(width)
    
    def setCalibrationF(self, factor):
        self._extruder.setCalibrationF(factor)

    def extrude(self, velocity: float, height: float):
        self._extruder.extrude(velocity, height)

    @abstractmethod
    def run(self):
        pass

    def runTasks(self, taskQueue):
        pass

    @abstractmethod
    def initial_conditions(self):
        pass    

    @abstractmethod
    def calibrate(self, calibrationPoints = []):
        pass
    
    @abstractmethod
    def programmStart(self):
        pass

    def saveHistory(self, dir = '.'): # TODO: update this
        self.TCPHistory = np.array(self.TCPHistory)
        self.jointHistory = np.array(self.jointHistory)
        logging.info(f"Saving robot history into {dir}/*History.npy")
        np.save(f"{dir}/TCPHistory.npy", self.TCPHistory)
        np.save(f"{dir}/jointHistory.npy", self.jointHistory)

    def disconnect(self):
        self._connection.disconnect()

    @classmethod
    def new_controller(cls, robot_data: dict, extruder_data: dict, dbaccess = None) -> Optional['Controller']:
        if extruder_data.get("extruder_type") is ToolType.Stoneflower:
            extruder = Stoneflower3D(extruder_data["host"])
        elif extruder_data.get("extruder_type") is ToolType.WASPclay:
            extruder = WASPclay(extruder_data["host"], extruder_data["port"], auth=extruder_data.get("auth", None))
        else:
            extruder = None

        if robot_data["robot_type"] in {1, 2, 3}:
            robotCon = URconnection(robot_data["host"], robot_data["port"], frequency=robot_data["frequency"], tsdb=dbaccess)
            return cls(robotCon, extruder)
        elif robot_data["robot_type"] in {4, 5}:
            robotCon = KUKAconnectionKRC4(robot_data["host"], robot_data["port"], True)
            return cls(robotCon, extruder)
        else:
            return None
