from machines.machinetypes import *
from fimkin import Screwkincalc
from dataconnections.fimDataHandler import PathData, GraphImporter
from dotenv import dotenv_values

config = dotenv_values(".env")

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

params = {
    'layerTime': 30,               # Layer time in seconds
    'calibrationFactor': 2.5,       # Calibration factor for extrusion
    'maxAcceleration': 1.3,         # Enable acceleration limiting -> max Acceleration in m/s^2
    'maxRobotAcceleration': 0.005,  # robot acceleration
    'scale': 0.2,
    'filamentWidth': 0.006,         # targed width of extruded filament in meters
    'limitLayers': 2,               # amount of layers to print
    'study': "TEST",
    'exp': 'EXP_CODE'
}

solver = Screwkincalc(2, 0.05)
pd = PathData(solver, 50)
pd.setConditions(**params)

fimImporter = GraphImporter(dbaccess["fimgraph"])
fimImporter.setPathData(pd)
fimImporter.initiateImport()

fimImporter.insert_camera_task()