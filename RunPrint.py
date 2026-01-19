import os
import numpy as np
from tkinter import filedialog as fd
from datetime import datetime 
import logging
import threading
from dotenv import dotenv_values
from queue import Queue

from machines import URC, RobotType, ToolType, Importer
from dataconnections.fimDataHandler import GraphImporter, FileImporter
from fimkin import Screwkincalc

def mainfunc():
    config = dotenv_values(".env")

    #region --Experiment constants--
    DEBUG = False

    ROBOT_TYPE = RobotType.UR10e
    EXTRUDER_TYPE = ToolType.WASPclay    
    IMPORTER_TYPE = Importer.Graph
    freq = 50 # Hz
    #endregion

    #region --Process and connection parameters--
    dbaccess = {
        "fimfile": r'./ifcFiles',
        "fimgraph": {
            "uri": config.get('NEO4J_URI'),
            "auth": tuple(config.get('NEO4J_AUTH').split(' '))
            },
        "influx": {
            "host": config.get('INFLUX_HOST'),
            "token": config.get('INFLUX_TOKEN'),
            "org": config.get('INFLUX_ORG'),
            "database": config.get('INFLUX_DATABASE')
            }
    }

    robot_data = {
        "robot_type": ROBOT_TYPE,
        "host": config.get('ROBOT_HOST'),
        "port": int(config.get('ROBOT_PORT')),
        "frequency": freq
    }

    extruder_data = {
        "extruder_type": EXTRUDER_TYPE,
        "host": config.get('EXTRUDER_HOST'),
        "port": int(config.get('EXTRUDER_PORT')),
        "auth": {
            "username": config.get('EXTRUDER_USER', ''),
            "password": config.get('EXTRUDER_PASS', '')
        }
    }

    params = {
        'A_curvature'      : 3.0,    # [m/s^2] Enable acceleration limiting in curves
        'A_fab'            : 0.02,    # [m/s^2] regular robot acceleration
        'A_travel'         : 0.2,    # [m/s^2] faster acceleration for transitions
        'F_rtde'           : freq,     # [Hz] Frequency of RTDE communication
        'N_layers'         : 2,      # [-] amount of layers to print
        'T_layerMAX'       : 300.0,  # [s] maximum Layer time in seconds
        'T_layerMIN'       : 5.0,    # [s] minimum Layer time in seconds
        'V_angular'        : 0.2,    # [rad/s] angular velocity for orientation changes
        'V_fab'            : 0.05,   # [m/s] regular fabrication speed
        'V_travel'         : 0.15,    # [m/s] travel speed
        'W_filament'       : 0.008,  # [m] targed width of extruded filament in meters
        'scaleFactor'      : 0.2,    # [-] Scale factor for the entire model
        'calibrationFactor': 3.0,    # Calibration factor for extrusion
        'layerTime'        : 10.0,   # [s] target time per layer
    }
    #endregion 

    #region --Run directory and logger setup--
    logging.basicConfig(level=logging.INFO,handlers=[logging.StreamHandler()])
    logger = logging.getLogger()
    formatter = logging.Formatter("%(asctime)s : %(levelname)s : %(message)s [%(filename)s %(lineno)d: %(funcName)s()]")
    logger.handlers[0].setFormatter(formatter)

    now = datetime.now()
    runDirectory = f'./runs/{now.strftime("%Y_%m_%d_%H_%M_%S")}'
    #runDirectory = fd.askdirectory(initialdir=f'./runs/')

    runDirectory = os.path.abspath(runDirectory)
    if runDirectory == os.path.abspath(f'.'):
        exit()

    if not os.path.exists(runDirectory):
        os.mkdir(runDirectory)
    else:
        logging.info(f"Run directory exists!! Will attempt to read preplanned path.")

    logfile = os.path.abspath(f"{runDirectory}/log.out")
    logFileHandler = logging.FileHandler(logfile, mode='w')
    logging.getLogger().addHandler(logFileHandler)
    logFileHandler.setFormatter(formatter)

    logging.info(f"Logging run to directory {logfile}")
    #endregion

    #region --Construction site definition--
    #TODO: implement selection of build space here

    platform = 0.4600
    table = 0.3559
    plattformLevel = -0.0235 + platform - table

    # Horizontal rectangle by two points
    P1 = np.array([0.9, 0.49, plattformLevel]).transpose()
    P0 = np.array([0.39, 0.25, plattformLevel]).transpose()
    #endregion

    #region --Machine and Data initialization--
    if   IMPORTER_TYPE is Importer.Graph:
        fimConnection = GraphImporter(dbaccess["fimgraph"])
    elif IMPORTER_TYPE is Importer.GraphOCC:
        # from fimocc import FIMgraphConnection
        # fimConnection = FIMgraphConnection(dbaccess["fimgraph"])
        # fimConnection.setGeometyPrecision(1E-5)
        raise NotImplementedError("FIMgraphConnection needs to be imported separately, see readme.")
    elif IMPORTER_TYPE is Importer.File:
        fimConnection = FileImporter({"uri": fd.askopenfilename(initialdir=dbaccess['fimfile'], title="Select IFC file", filetypes=[("IFC files", "*.ifc")])})
        if not fimConnection.ifcfile["uri"]:
            exit()
    else:
        raise ValueError("Importer type not recognized.")
    
    fimConnection.kinematicSolver = Screwkincalc(robot=robot_data["robot_type"], threshold=0.05, toolOffset=extruder_data.get("extruder_type").value)
    fimConnection.setConditions(**params)
    fimConnection.initiateImport()

    if IMPORTER_TYPE is Importer.GraphOCC:
        fimConnection.fit2buildPlate([P0, P1])
    else:
        fimConnection.calculateOffset([P0, P1])

    printer = URC.new_controller(robot_data, extruder_data, dbaccess["influx"])
    printer.setParameters(**params)
    #endregion

    #region --Initial position--
    if not DEBUG:
        printer._connection.initialize()
        joints0, xyz0 = printer.initial_conditions()
    else:
        # Home Position UR10e
        joints0 = [-3.141592653, -1.570796327, 1.570796327, -1.570796327, -1.370796327, -1.570796327]
        logging.info(f"[DEBUG = True] Will not connect to Robot for state 0")

    fimConnection.setInitialWaypointByJC(joints0)
    fimConnection.selectLayerTransitionPosition()
    #endregion

    #region --Parallelization--
    fimConnection.closeSession()

    if DEBUG:
        logging.info(f"[DEBUG = True] Will not connect to Robot for run")
        return 1
    tasks = Queue()
    taskexecuter = threading.Thread(target=printer.runTasks, args=(tasks,))
    
    taskexecuter.start()
    fimConnection.extractTasks(tasks)
    taskexecuter.join()
    print("All tasks finished.")

    logging.info(f"Run finished, documented in {runDirectory}")
    return 1

if __name__ == '__main__':
    mainfunc()
    