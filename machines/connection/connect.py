import logging

class failedToConnect(Exception):
    def __init__(self, message = "Failed to establish connection", robotIP = None, robotPort = None):
        if robotIP and robotPort:
            message = message + f" via IP: {robotIP} , Port: {robotPort}"
            logging.ERROR(message)
        self.message = message
        super().__init__(self.message)

class RobotConnection:
    def __init__(self, ROBOT_HOST, ROBOT_PORT, config_filename="", frequency = 125):
        self.configfile = config_filename
        self.robotIP = ROBOT_HOST
        self.robotPort = ROBOT_PORT
        self.frequency = frequency

    def connectTSDB(self):
        pass

    def initialize(self, watchdog:bool = False) -> None:    #To specific
        pass

    def connect(self) -> None:
        pass

    def sync(self) -> None: 	        #To specific
        pass

    def send(self, content, debug:bool = False) -> None:
        pass

    def send_watchdog(self) -> None:    #To specific
        pass

    def receive(self):
        return []
    
    def sendInflux(self, state) -> None:
        pass

    def disconnect(self) -> None:
        pass