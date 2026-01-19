import numpy as np
from machines.control.controller import Controller
import logging

logger = logging.getLogger(__name__)


class KRC(Controller):
    def __init__(self, robotConnection, extruder):
        super().__init__(robotConnection, extruder)
   
    def programmStart(self): # implementation necessary after KUKA Tests
        pass

    def programmStart(self): # implementation necessary
        pass

    def run(self): # TODO: needs a lot of fixes..
        # eventuell pop up?
    
        layer_height = 0.002
        # senden der Positionswerte 0,0,0,0,0,0
        self.path.jointsPath = np.array(self.path.jointsPath)
        try:
            while True:
                state = self._connection.read()
                
                if state.status ==1:
                    logging.info("Exiting first while loop.")
                    # self.sync()
                    break
            
                           
            for layer in self._pathData.getLayers():
                for waypoint in layer.getWaypoints(): # more changes necessary
                    if not self.debug:
                        state = self._connection.read()
                        logging.debug(f"Received {state}") # x: aktuelle KUKA Struktur
                        
                        if isinstance(state, type(None)):
                            logging.critical(f"Current state recieved from robot is NoneType.")
                            return 
                        
                        
                    if state.status == 1:
                        logging.info(f"Current State status = { state.status }")
                        self.TCPHistory = np.append(self.TCPHistory, np.array([state.pos]), axis=0)
                        self.jointHistory = np.append(self.jointHistory, np.array([state.joints]), axis=0)
                        # -----
                        t = self.path.t[self.ptCounter]
                        nextPt = self.path.jointsPath[self.ptCounter,:]
                        logging.info(f" Counter =  {self.ptCounter}, t = {t}, DOF = {nextPt}.")
                        if np.linalg.norm(self.jointHistory[-1] - nextPt) > 1: 
                            logging.info(f"Excessive joint movement requested! aborting")
                            break
                        if not self.debug:
                            point_to_send = nextPt.tolist()
                            point_to_send.append(1)
                            self._connection.send(nextPt.tolist())
                        # --- Extruder 
                        if not isinstance(self.extruder_serial_port, type(None)):
                            if self.ptCounter % 10 == 0:
                                self.previousExtruderSpeed = self.extruder_serial_func(self.extruder_serial_port, self.path.velProfile[self.ptCounter], layer_height, self.previousExtruderSpeed, self.extruder_speed_expre)
                        

        
                
                else:
                    
                    if state.status != self.lastState:
                        logging.info(f"Current Status {state.status}")
                    self.lastState = state.status
                    
                
            if not isinstance(self.extruder_serial_port, type(None)):
                self.previousExtruderSpeed = self.extruder_serial_func(self.extruder_serial_port, 0, layer_height, 80000)
        except KeyboardInterrupt:
            if not isinstance(self.extruder_serial_port, type(None)):
                self.previousExtruderSpeed = self.extruder_serial_func(self.extruder_serial_port, 0, layer_height, 80000)
        finally:
            if not isinstance(self.extruder_serial_port, type(None)):
                self.previousExtruderSpeed = self.extruder_serial_func(self.extruder_serial_port, 0, layer_height, 80000)

                    
    #changes necessary
    def initial_conditions(self):
        #initial joint and xyz position of the robot
        act_pos = self._connection.read("$Axis_ACT_MEANS") #Format tbd
        self.disconnect()
        return act_pos
    
