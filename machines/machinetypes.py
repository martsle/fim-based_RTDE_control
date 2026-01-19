from enum import IntEnum, Enum, auto

class RobotType(IntEnum):
    UR5e  = 1
    UR10e = 2
    UR16e = 3
    KR120 = 4
    KR300 = 5

class RobotClass(set, Enum):
    UR   = {RobotType.UR5e, RobotType.UR10e, RobotType.UR16e}
    KUKA = {RobotType.KR120, RobotType.KR300}

class ToolType(list, Enum):
    def __getitem__(self, index):
        return self._value_[index]
    Stoneflower = [0.0, 0.0, 0.2000]
    WASPclay = [0.0, 0.0, 0.3798]
    RealSense = [0.0, 0.0, 0.1350] #TODO: measure!

class Importer(Enum):
    File = 0
    Graph = 1
    GraphOCC = 2