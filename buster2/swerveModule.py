import ctre
from wpimath import kinematics, geometry
import math
from ctre import AbsoluteSensorRange, SensorInitializationStrategy
from controlsystems.diagnostics import Diagnostics
from wpilib import SmartDashboard

class SwerveModule:
    countsPerRotation = 2048 # encoder ticks per rotation
    turningGearRatio = 12.8 # 12.8 motor spins per wheel spin
    drivingGearRatio = 8.14 # 8.14 motor spins per wheel spin
    wheelDiameter = 0.1010 # in meters
    motorEncoderConversionFactor = 2 * math.pi / countsPerRotation * 12.8
    def __init__(self, config: dict, moduleName: str) -> None:
        self.moduleName = moduleName
        
        self.driveMotor = ctre.TalonFX(config["driveMotorID"])
        self.turnMotor = ctre.TalonFX(config["turnMotorID"])
        self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 250)
        self.turnMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 250)
        
        self.absoluteEncoder = ctre.CANCoder(config["encoderID"])
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 250)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, 250)
        self.absoluteEncoder.configMagnetOffset(-config["encoderOffset"], 250)
        
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.turnMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero)
        
    def testPeriodic(self):
        SmartDashboard.putNumber(f"{self.moduleName} Relative:", self.turnMotor.getSelectedSensorPosition(0))
        SmartDashboard.putNumber(f"{self.moduleName} Absolute:", self.absoluteEncoder.getAbsolutePosition())