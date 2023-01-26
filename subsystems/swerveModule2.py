import ctre
from wpimath import kinematics, geometry
import math
from ctre import AbsoluteSensorRange, SensorInitializationStrategy
from controlsystems.diagnostics import Diagnostics

class SwerveModule:
    countsPerRotation = 2048 # encoder ticks per rotation
    turningGearRatio = 12.8 # 12.8 motor spins per wheel spin
    drivingGearRatio = 8.14 # 8.14 motor spins per wheel spin
    wheelDiameter = 0.1010 # in meters
    motorEncoderConversionFactor = 2 * math.pi / countsPerRotation * 12.8
    def __init__(self, config: dict, dianostics: Diagnostics) -> None:
        pass