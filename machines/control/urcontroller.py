import numpy as np
import logging
import queue
import time
import traceback
from machines.control.controller import Controller
from multiprocessing import Queue, Process, Value
from multiprocessing.sharedctypes import Synchronized
from dataconnections.fimDataHandler import Move, Event, Extrude, UseSensor
from sensors import ZED

logger = logging.getLogger(__name__)

class URC(Controller):
    def __init__(self, _connectionnection, extruder):
        super().__init__(_connectionnection, extruder)
        self._actualSpeed = 0.0
        self._wpSpeed = 0.0

    def programmStart(self):
        
        state = self._connection.receive()
        if state.runtime_state == 2:
            print("Program is running")
        else:
            print('Waiting for program start')
            while True:
                state = self._connection.receive()
                if state.runtime_state == 2:
                    print("Program is running")
                    break
            print('Waiting...')
            while True:
                state = self._connection.receive()
                if state.output_int_register_0 == 1:
                    print("output_int_register_0 = 1")
                    break 

        print('Waiting for init')
        while True:
            state = self._connection.receive()
            if state.output_int_register_0 == 0:
                print("output_int_register_0 = 0")
                break 

        self._connection.send([0,0,0,0,0,0,0])
        print("send [0,0,0,0,0,0]")

    def goToWaypoint(self, waypoint, state: Queue):
        if not self._debug:
            self.currentState = self._connection.receive()
            state.put(self.currentState)
            logging.debug(f"Received {self.currentState}")
            if isinstance(self.currentState, type(None)):
                logging.critical(f"Current state recieved from robot is NoneType.")
                return 
        if self.currentState.runtime_state == 2: # and currState.output_int_register_0 != 0:
            logging.info(f"Current State runtime_state = {self.currentState.runtime_state} | currState.output_int_register_0 { self.currentState.output_int_register_0 }")
            self.TCPHistory = np.append(self.TCPHistory, np.array([self.currentState.actual_TCP_pose]), axis=0)
            self.jointHistory = np.append(self.jointHistory, np.array([self.currentState.actual_q]), axis=0)
            speedVec = np.array([self.currentState.target_TCP_speed])
            self._actualSpeed = np.linalg.norm(speedVec[0:3])
            self._wpSpeed = waypoint._velocity

            nextPt = waypoint.getJointangles()
            #logging.info(f" Counter =  {self.ptCounter}, t = {t}, DOF = {nextPt}.")
            if np.linalg.norm(self.jointHistory[-1] - nextPt) > 1: 
                logging.info(f"Excessive joint movement requested! aborting")
                raise ValueError
            if not self._debug:
                point_to_send = nextPt.tolist()
                point_to_send.append(1)
                self._connection.send(point_to_send)

            self.ptCounter += 1
            self._connection.send_watchdog()
            #self._connection.sendInflux(self.currentState)
        else:
            self._connection.send_watchdog()
            if self.currentState.output_int_register_0 != self.lastRegis:
                logging.info(f" currState.output_int_register_0 { self.currentState.output_int_register_0 }")
            if self.currentState.runtime_state != self.lastState:
                logging.info(f"Current State runtime_state = {self.currentState.runtime_state} | currState.output_int_register_0 { self.currentState.output_int_register_0 }")
            self.lastState = self.currentState.runtime_state
            self.lastRegis = self.currentState.output_int_register_0
        
    def run(self):
        self._connection.initialize()
        self._extruder.connect()
        self._connection.send_watchdog()
        self.programmStart()
    
        try:
            for waypoint in self._pathData._discretizedTransition:
                self.goToWaypoint(waypoint)

            for layer in self._pathData.layers():
                for n, waypoint in enumerate(layer.getWaypoints()):
                    self.goToWaypoint(waypoint)
                    #if n%5 == 2:
                    self.extrude(self._wpSpeed,layer.layerHeight)
        finally:
            self._connection.disconnect()
            self._extruder.stop()
    
    def runExtruder(self, height: Synchronized, speed: Synchronized, terminate: Synchronized, dt: float= 0.2):
        self._extruder.connect()
        while True:
            try:
                with terminate.get_lock():
                    if terminate.value:
                        break
                if height.value and speed.value:
                    self.extrude(speed.value, height.value)
                else:
                    self._extruder.stop()
                    while not height.value or not speed.value:
                        time.sleep(dt/100)
                time.sleep(dt-0.005)
            except Exception as err:
                traceback.print_tb(err.__traceback__)
                time.sleep(1)
        self._extruder.disconnect()


    def communicateJob(self, jobqueue: Queue, speed: Synchronized, moveStarted: Synchronized, terminate: Synchronized, state: Queue):
        self._connection.initialize()
        self._connection.send_watchdog()
        self.programmStart()

        while True:
            try:
                with terminate.get_lock():
                    if terminate.value:
                        break
                if moveStarted.value:
                    job = jobqueue.get(timeout=.5)

                    for wp in job:
                        self.goToWaypoint(wp, state)
                        with speed.get_lock():
                            speed.value = self._wpSpeed
                    with moveStarted.get_lock():
                        moveStarted.value = False
                    with speed.get_lock():
                        speed.value = 0.0
                else:
                    currentState = self._connection.receive()
                    state.put(currentState)
                    self._connection.send_watchdog()
                    time.sleep(1/self._connection.frequency)
            except queue.Empty:
                currentState = self._connection.receive()
                state.put(currentState)
                self._connection.send_watchdog()
                print("Queue empty! Waiting...")

    def handleTSD(self, state:Queue, terminate: Synchronized):
        self._connection.connectTSDB()
        while True:
            with terminate.get_lock():
                if terminate.value:
                    break
            try:
                currstate = state.get(timeout=.5)
                self._connection.sendInflux(currstate)
            except queue.Empty:
                pass
            except Exception as err:
                traceback.print_tb(err.__traceback__)
                print(repr(err))


    def runTasks(self, taskQueue:Queue):
        # zedHandler = ZED()
        # zedHandler.connect_camera()
        # zedHandler.check_and_switch_dir(self._study, self._exp)
        jobqueue = Queue()
        state = Queue()
        speed = Value('f', 0.0)
        extrusionHeight = Value('f', 0.0)
        moveStarted = Value('b', False)
        terminate = Value('b', False)
        robotWorker = Process(target=self.communicateJob, args=(jobqueue, speed, moveStarted, terminate, state))
        extruderWorker = Process(target=self.runExtruder, args=(extrusionHeight, speed, terminate))
        influxWorker = Process(target=self.handleTSD, args=(state, terminate))
        robotWorker.start()
        extruderWorker.start()
        influxWorker.start()

        activated = True
        while activated:
            try:    
                currentTask = taskQueue.get(timeout=.9)
                # if type(currentTask).__name__ is 'Event':
                #     if currentTask.Name == "End" and taskQueue.empty():
                #         break
                #     continue
                
                for actn in currentTask:
                    if type(actn).__name__ == 'Move':
                        with moveStarted.get_lock():
                            moveStarted.value = True
                        jobqueue.put(actn.job)
                    if type(actn).__name__ == 'Extrude':
                        with extrusionHeight.get_lock():
                            extrusionHeight.value = actn.job
                    if type(actn).__name__ == 'UseSensor':
                        time.sleep(0.5)
                        # zedHandler.take_scan(actn.job)
                        time.sleep(0.5)
                    if type(actn).__name__ == 'EndProcess':
                        activated = False
                        with moveStarted.get_lock():
                            moveStarted.value = False
                        with terminate.get_lock():
                            terminate.value = True
                while moveStarted.value:
                    time.sleep(1/self._connection.frequency)

                with extrusionHeight.get_lock():
                    extrusionHeight.value = 0.0
            
                print(f"Finished task normally.")
            except queue.Empty:
                print("Waiting for task...")
            except Exception as err:
                traceback.print_tb(err.__traceback__)
        
        self._extruder.connect()
        self._extruder.stop()
        extruderWorker.join()
        robotWorker.join()
        influxWorker.join()

        print("All processes finished.")
        return
        # self._connection.disconnect()
        # zedHandler.disconnect()


    def calibrate(self, calibrationPoints = []):
        calibratedPoints = []
        self._connection.send_watchdog()
        self.programmStart()
        
        state = self._connection.receive()
        if state.runtime_state == 2:
            print("Program is running")
        else:
            print('Waiting for program start')
            while True:
                state = self._connection.receive()
                if state.runtime_state == 2:
                    print("Program is running")
                    break
            print('Waiting...')
            while True:
                state = self._connection.receive()
                if state.output_int_register_0 == 1:
                    print("output_int_register_0 = 1")
                    break 

        
        print('Waiting for init')
        while True:
            state = self._connection.receive()
            if state.output_int_register_0 == 0:
                print("output_int_register_0 = 0")
                break 

        self._connection.send([0,0,0,0,0,0,0])
        print("send [0,0,0,0,0,0]")
        
        try:
            for point in calibrationPoints:
                point.append(1)
              
                self._connection.send(point)
                print("Command = 1")
                self._connection.sync()   
                # waiting for acknowledgement
                while True:
                    self._connection.send_watchdog()
                    state = self._connection.receive()
                    if state.output_int_register_0 == 1: 
                        logging.info(f"Robot has received point {point}")
                        break
                point[6] = 0
                self._connection.send(point) # Used for safty, should not be readed by the robot. 0 stands for 'do not move',
                print("Command = 0")

                # waiting for collision
                while True:
                    self._connection.send_watchdog()
                    self._connection.sync()
                    state = self._connection.receive()
                    if state.output_int_register_0 == 2: 
                        print("Collision detected")
                        break
                self._connection.send_watchdog()
                state = self._connection.receive()
                print(f"Received TCP Pose: {type(state.actual_TCP_pose)}: {state.actual_TCP_pose}")
                command2_state = point
                command2_state[6]=2
                self._connection.send(command2_state)
                print(f"send {command2_state}")
                print("Command=2")
                calibratedPoints.append(state.actual_TCP_pose)
                print("Calibration Point appended")

        #except RobotStopException:
            

        # except KeyboardInterrupt:
        #     if not isinstance(self.extruder_serial_port, type(None)):
        #         self.extruder_serial_func(self.extruder_serial_port, 0, 1)
        finally:
            pass
            # if not isinstance(self.extruder_serial_port, type(None)):
            #     self.extruder_serial_func(self.extruder_serial_port, 0, 1)

        return calibratedPoints
    
    def initial_conditions(self):
        state = self._connection.receive()
        joints0 = state.actual_q
        logging.info(f"Joints at t = 0 : {joints0}")
        xyz0 = state.actual_TCP_pose            
        logging.info(f"Positions at t = 0 : {xyz0}")
        self._connection.disconnect()
        return joints0, xyz0
    