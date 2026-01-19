import numpy as np
from machines.control.controller import Controller
import logging

logger = logging.getLogger(__name__)


class KRC(Controller):
    def __init__(self, robotConnection, extruder):
        super().__init__(robotConnection, extruder)
   
    def programmStart_initial_package(self): 
        while True:
            if self._connection.readVar("$RUNNING") == "1":
                print("Program is running.")
                break
        
        if self._pathData._discretizedTransition.length() >= 5:
            init_package = self._pathData._discretizedTransition[:5]
            del self._pathData._discretizedTransition[:5]  
        else:
            init_package = self._pathData._discretizedTransition
            init_package.extend(self._pathData.layers()[0].getWaypoints()[:5-self._pathData._discretizedTransition.length()] )
        
        self._connection.send_init_package(init_package)

    def programmStart(self): 
        while True:
            if self._connection.readVar("$RUNNING") == "1":
                print("Program is running.")
                break
        

    def goToWaypoint(self, waypoint, last_waypoint):
        if not self._debug:
            currState = self._connection.receive()##
            logging.debug(f"Received {currState}")
            if isinstance(currState, type(None)):
                logging.critical(f"Current state recieved from robot is NoneType.")
                return 
        self.TCPHistory = np.append(self.TCPHistory, np.array([currState.actual_TCP_pose]), axis=0)
        nextPt = waypoint.getPose()
        #logging.info(f" Counter =  {self.ptCounter}, t = {t}, DOF = {nextPt}.")
        #if np.linalg.norm(self.jointHistory[-1] - nextPt) > 1: 
            #logging.info(f"Excessive joint movement requested! aborting")
            #raise ValueError
        if not self._debug:
            # Change with regard to the test result if necessary
            point_to_send = nextPt.tolist()
            point_to_send.append(self.calculate_velocity(last_waypoint, waypoint))
            self._connection.send(point_to_send)
        # --- Extruder 
        #if not isinstance(self.extruder_serial_port, type(None)):
            #if self.ptCounter % 10 == 0:
                #self.extruder_serial_func(self.extruder_serial_port, self.path.velProfile[self.ptCounter], layer_height, self.extruder_speed_expre)
                #self.extruder_serial_func(self.extruder_serial_port, self.path.velProfile[self.ptCounter], self.path.heights[self.ptCounter], self.extruder_speed_expre)
        self.ptCounter += 1

    def run(self):

        layer_height = 0.002
        
        for layer in self._pathData._layers:
            jPath = 0

        #self.path.jointsPath = np.array(self.path.jointsPath) # necessary?
        #self.path.heights = np.array(self.path.heights) # necessary?
        self.programmStart() # can be changed to programmStart_initial_package
        	
        
        try:

            self.writeVar('$START_Command', str(1), debug=False)
            last_waypoint = self._connection.receive()
            for waypoint in self._pathData._discretizedTransition:
                self.goToWaypoint(waypoint, last_waypoint)
                last_waypoint = waypoint

            for layer in self._pathData.layers():
                for waypoint in layer.getWaypoints():
                    self.goToWaypoint(waypoint, last_waypoint)
                    last_waypoint = waypoint
                        
                    

        except KeyboardInterrupt: # TODO: this is already in finally...
            self._extruder.stop()
        finally:
            self._extruder.stop()
            
    def test(self):
        POS_ACT = self._connection.receive()
        print(POS_ACT)
        print(type(POS_ACT))
        for element in POS_ACT:
            print(type(element))
                        
    def initial_conditions(self):
        #initial joint and xyz position of the robot
        act_pos = self._connection.receive() #Format tbd
        self.disconnect()
        return act_pos
    
    def calculate_velocity(self, start, end):
        if isinstance(start, list):
            double_start = [float(element) for element in start]
            distance = np.linalg.norm(double_start[:3] - end._cartesian)
            velocity = distance*125
            return velocity

        distance = np.linalg.norm(start._cartesian - end._cartesian)
        velocity = distance*125
        return velocity
    
