import ctre
from wpimath import kinematics, geometry
import math
from ctre import AbsoluteSensorRange, SensorInitializationStrategy
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
        self.turnMotor.setSelectedSensorPosition(self.absoluteEncoder.getAbsolutePosition() * 2048 * 12.8 / 360, 0, 250)
        self.turnMotor.config_kP(0, 0.1, 250)
        self.turnMotor.config_kI(0, 0.0001, 250) #0.0001
        self.turnMotor.config_kD(0, 0.0001, 250)
        self.turnMotor.config_kF(0, 0, 250)
        self.turnMotor.config_IntegralZone(0, 100, 250)
        self.turnMotor.configAllowableClosedloopError(0, 0, 250)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Coast)
        
        self.driveMotor.config_kP(0, 0.0005, 250)
        self.driveMotor.config_kI(0, 0.0005, 250) # 0.0005
        self.driveMotor.config_kD(0, 0, 250)
        self.driveMotor.config_kF(0, 0.045, 250)
        self.driveMotor.config_IntegralZone(0, 500, 250)
        self.driveMotor.configAllowableClosedloopError(0, 250)
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Coast)
        
    def getWheelAngleRadians(self):
        adjustedRelValDegrees = ((self.turnMotor.getSelectedSensorPosition(0) % (2048 * 12.8)) * 360) / (2048 * 12.8) # zero to 360 scale
        adjustedRelValDegrees -= 180 # adjusting it to be 180/-180
        if adjustedRelValDegrees < -180:
            adjustedRelValDegrees = 360 + adjustedRelValDegrees
        return math.radians(adjustedRelValDegrees) # converting the value to radians and returning it
    
    def getTurnWheelState(self):
        return geometry.Rotation2d(self.getWheelAngleRadians())
        
    def testPeriodic(self):
        adjustedRelValDegrees = ((self.turnMotor.getSelectedSensorPosition(0) % (2048 * 12.8)) * 360) / (2048 * 12.8)
        adjustedRelValDegrees -= 180
        if adjustedRelValDegrees < -180:
            adjustedRelValDegrees = 360 + adjustedRelValDegrees
        SmartDashboard.putNumber(f"{self.moduleName} Relative:", adjustedRelValDegrees)
        SmartDashboard.putNumber(f"{self.moduleName} Absolute:", self.absoluteEncoder.getAbsolutePosition())
        
    def getSwerveModuleState(self):
        distanceMeters = self.driveMotor.getSelectedSensorPosition(0) * self.wheelDiameter * math.pi / self.countsPerRotation * self.drivingGearRatio # converting the clicks into distance values, in this case, meters
        angle = self.getTurnWheelState()
        return kinematics.SwerveModulePosition(distanceMeters, angle)
            
    def setState(self, state: kinematics.SwerveModuleState):
        state = kinematics.SwerveModuleState.optimize(state, self.getTurnWheelState())
        desiredStateAngle = state.angle.degrees()
        if desiredStateAngle > 0:
            desiredStateAngle -= 180
        else:
            desiredStateAngle += 180
        state.angle = geometry.Rotation2d(math.radians(desiredStateAngle))
        #SmartDashboard.putNumber(f"{self.moduleName} targetState:", state.angle.degrees())
        velocity = (state.speed * self.drivingGearRatio * self.countsPerRotation) / (self.wheelDiameter * math.pi * 10) # converting from meters per second to ticks per hundred milliseconds
        self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity)
        
        desiredAngleDegrees = state.angle.degrees()
        if desiredAngleDegrees < 0:
            desiredAngleDegrees += 360
        motorEncoderTickTarget = (self.turnMotor.getSelectedSensorPosition(0) - (self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio))) + desiredAngleDegrees * self.countsPerRotation * self.turningGearRatio / 360
        self.turnMotor.set(ctre.TalonFXControlMode.Position, motorEncoderTickTarget)
        
    def setNeutralMode(self, mode):
        self.driveMotor.setNeutralMode(mode)
        self.turnMotor.setNeutralMode(mode)