import ctre
from wpimath import kinematics, geometry
import math
from ctre import AbsoluteSensorRange, SensorInitializationStrategy
from controlsystems.diagnostics import Diagnostics

class SwerveModule:
    
    def __init__(self, config: dict, dianostics: Diagnostics) -> None:
        pass