import os
from rtde import rtde, rtde_config
from machines.connection.connect import *
from influxdb_client_3 import InfluxDBClient3, Point, WritePrecision, SYNCHRONOUS
from datetime import datetime

logger = logging.getLogger(__name__)
package_directory = os.path.dirname(os.path.abspath(__file__))


class URconnection(RobotConnection):
    def __init__(self, ROBOT_HOST, ROBOT_PORT, config_filename=package_directory+"/urconnect_configuration.xml", frequency = 125, tsdb = None):
        super().__init__(ROBOT_HOST, ROBOT_PORT, config_filename, frequency)
        
        self.config = rtde_config.ConfigFile(config_filename)
        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.tsdbconnection = tsdb
        self.tsdb = None
        #self.tsdb: InfluxDBClient3 = InfluxDBClient3(**tsdb) if not isinstance(tsdb, type(None)) else None
        
        # recipes:
        self.recipe = {}
        self.recipe["state"] = self.config.get_recipe('state')
        self.recipe["setp"] = self.config.get_recipe('setp')
        self.recipe["watchdog"] = self.config.get_recipe('watchdog')
        logger.debug(f"Config file {self.configfile} used for connection configuration.")
        
        # DataObjects:
        self.setp = None
        self.watchdog = None

    def connectTSDB(self):
        assert not isinstance(self.tsdbconnection, type(None)), "TSDB connection defined."
        self.tsdb = InfluxDBClient3(**self.tsdbconnection)
        logging.info(f"Connected to InfluxDB.")

    def initialize(self, watchdog:bool = True) -> None:
        self.connect()
        self.con.get_controller_version()

        # setup recipes
        logging.debug(f"Output setup: ")
        for z in list(zip(*self.recipe["state"])):
            logging.debug(f" - {z[0],z[1]}")
        self.con.send_output_setup(*self.recipe["state"], frequency = self.frequency) # This has a standard freq of 125, change to 500HZ
        self.setp = self.con.send_input_setup(*self.recipe["setp"])
        if watchdog:
            self.watchdog = self.con.send_input_setup(*self.recipe["watchdog"])

        # We can add more registers, all must be initialized
        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0
        self.setp.input_double_register_6 = 0
        logging.info(f"input_double_register_i initialized to 0, i = 0 -> 5.")
        # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
        
        self.sync()
        if watchdog:
            self.watchdog.input_int_register_0 = 0
            self.send_watchdog()
        
        logging.info(f"input_int_register_0 initialized to 0.")
        self.sync()

    def connect(self):
        self.con.connect()
        logging.info(f"Connection established via {self.robotIP} {self.robotPort}.")
    
    def sync(self):
        if not self.con.send_start():
            raise failedToConnect
        logging.info(f"Connection synced.")

    def send(self, content, debug = False):
        if isinstance(content, list):
            content = self.list_to_setp(content)
        if debug:
            logging.info(f"[TRIAL/notSent] Sent {content},{type(content)}.")
        else:
            logging.debug(f"Sending {content},{type(content)}.")
            self.con.send(content)
            logging.debug(f"Sent.")
    
    def send_watchdog(self):
        self.con.send(self.watchdog)
    
    def sendInflux(self, state):
        if not isinstance(self.tsdb, type(None)):
            ts = int(datetime.now().timestamp() * 1000)
            actual_TCP = (
                Point("actual_TCP_pose")
                .tag("robot", "UR10e")
                .field("X", state.actual_TCP_pose[0])
                .field("Y", state.actual_TCP_pose[1])
                .field("Z", state.actual_TCP_pose[2])
                .field("Rx", state.actual_TCP_pose[3])
                .field("Ry", state.actual_TCP_pose[4])
                .field("Rz", state.actual_TCP_pose[5])
                .time(ts, WritePrecision.MS)
            )
            actual_q = (
                Point("actual_q")
                .tag("robot", "UR10e")
                .field("base", state.actual_q[0])
                .field("shoulder", state.actual_q[1])
                .field("elbow", state.actual_q[2])
                .field("wrist1", state.actual_q[3])
                .field("wrist2", state.actual_q[4])
                .field("wrist3", state.actual_q[5])
                .time(ts, WritePrecision.MS)
            )
            actual_TCP_speed = (
                Point("actual_TCP_speed")
                .tag("robot", "UR10e")
                .field("v_X", state.actual_TCP_speed[0])
                .field("v_Y", state.actual_TCP_speed[1])
                .field("v_Z", state.actual_TCP_speed[2])
                .field("v_Rx", state.actual_TCP_speed[3])
                .field("v_Ry", state.actual_TCP_speed[4])
                .field("v_Rz", state.actual_TCP_speed[5])
                .time(ts, WritePrecision.MS)
            )
            actual_joint_voltage = (
                Point("actual_joint_voltage")
                .tag("robot", "UR10e")
                .field("v_base", state.actual_joint_voltage[0])
                .field("v_shoulder", state.actual_joint_voltage[1])
                .field("v_elbow", state.actual_joint_voltage[2])
                .field("v_wrist1", state.actual_joint_voltage[3])
                .field("v_wrist2", state.actual_joint_voltage[4])
                .field("v_wrist3", state.actual_joint_voltage[5])
                .time(ts, WritePrecision.MS)
            )
            actual_current = (
                Point("actual_current")
                .tag("robot", "UR10e")
                .field("i_base", state.actual_current[0])
                .field("i_shoulder", state.actual_current[1])
                .field("i_elbow", state.actual_current[2])
                .field("i_wrist1", state.actual_current[3])
                .field("i_wrist2", state.actual_current[4])
                .field("i_wrist3", state.actual_current[5])
                .time(ts, WritePrecision.MS)
            )
            self.tsdb.write([actual_TCP, actual_q, actual_TCP_speed, actual_joint_voltage, actual_current])
            logging.debug(f"Sent to InfluxDB.")
        else:
            logging.debug(f"TSDB not initialized, not sending to InfluxDB.")

    def receive(self):
        content = self.con.receive()
        logging.debug(f"Received {content}, {type(content)}.")
        return content

    def disconnect(self):
        self.con.send_pause()
        logging.info(f"Connection paused.")
        self.con.disconnect()
        logging.info(f"Connection disconnected successfuly.")

    def setp_to_list(self):
        list = []
        for i in range(0,7):
            list.append(self.setp.__dict__["input_double_register_%i" % i])
        return list

    def list_to_setp(self, list):
        for i in range (0,7):
            self.setp.__dict__["input_double_register_%i" % i] = list[i]
        return self.setp