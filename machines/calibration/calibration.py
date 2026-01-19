import sys
sys.path.append('.')
from control import * 
from control.setup_robot import new_controller

class Calibration(object):
    def __init__(self, init_y, init_x, height, width, curved_Surface, curved_height):
        self.init_y = 0 - init_y
        self.init_x = 0 - init_x
        self.height = height - 20.0
        self.width = width - 20.0
        self.toolOffset = 146.0 #+ 80
        self.offset = -29 + 20 + self.toolOffset if not curved_Surface else 15 + curved_height +self.toolOffset
        print(f"offset {self.offset}")
        self.caliPoints = []
        self.calibratedPoints = []
        self.h, self.w = self.numberCalibrationPoints()
        self.calculateCalibrationPoints()
        self.controller = self.getController()
        print("Calibration Initialized")

    def getController(self):
        # Modularisation TBD
        freq = 125 # Hz
        ROBOT_HOST = '10.152.49.60' # IP Address of the robot
        ROBOT_PORT = 30004
        config_filename = './execution/control_loop_configuration.xml'
        robot_type ="ur10e"

        return new_controller(robot_type, ROBOT_HOST, ROBOT_PORT, freq, config_filename)
    def numberCalibrationPoints(self):
        height = self.height
        width = self.width
        distance = 100.0
        h = int(height / distance) + 1
        w = int(width / distance) + 1
        return h, w

    def calculateCalibrationPoints(self):
        distance_x = self.width / (self.w - 1)
        distance_y = self.height / (self.h - 1)

        for n in range(self.w):
            for m in range(self.h):
                point = [(self.init_x - 10.0 - distance_x * n)/1000, (self.init_y - 10.0 - distance_y * m)/1000, (self.offset)/1000, 0.0, 3.1415, 0.0] 
                self.caliPoints.append(point)
        self.caliPoints.reverse()
            
    def calibrate(self):
        self.calibratedPoints = self.controller.calibrate(self.caliPoints)
    
    