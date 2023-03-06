import ctre
from wpimath import kinematics, geometry 
import wpilib
import math
from ctre.sensors import AbsoluteSensorRange, SensorInitializationStrategy
from wpilib import SmartDashboard, AnalogEncoder 

class SwerveModule:
    countsPerRotation = 2048 # encoder ticks per rotation
    turningGearRatio = 12.8 # 12.8 motor spins per wheel spin
    drivingGearRatio = 8.14 # 8.14 motor spins per wheel spin
    wheelDiameter = 0.1016 # in meters
    motorEncoderConversionFactor = 2 * math.pi / countsPerRotation * 12.8
    wheelVelocityThreshold = 0.1 # in meters per second, if we wheel is being told to spin any slower than this value, just coast the drive motor
    '''
    TODO:
    - optimize functions by using all radians instead of converting from ticks to degrees to radians and vice versa
    - add debugging statements to all motor configs, or at the very least the non-persistant settings ie: setting of offsets
    '''
    def __init__(self, config: dict, moduleName: str) -> None:
        self.moduleName = moduleName
        self.xAngle = config["xAngle"]
        wpilib.AnalogInput(config["encoderID"]).setSampleRate(125)
        self.driveMotor = ctre.TalonFX(config["driveMotorID"])
        self.turnMotor = ctre.TalonFX(config["turnMotorID"])
        self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 250)
        self.turnMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 250)
        
        self.absoluteEncoder = AnalogEncoder(config["encoderID"])
        self.absoluteOffset = config["encoderOffset"]
        #self.absoluteEncoder.setPositionOffset(config["encoderOffset"] / 360)
        
        turnMotorConfig = ctre.TalonFXConfiguration()
        turnMotorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
        turnMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero
        turnMotorConfig.slot0.kP = 0.1
        turnMotorConfig.slot0.kI = 0.0001
        turnMotorConfig.slot0.kD = 0.0001
        turnMotorConfig.slot0.kF = 0
        turnMotorConfig.slot0.integralZone = 100
        turnMotorConfig.slot0.allowableClosedloopError = 0
        turnSupplyCurrentConfig = ctre.SupplyCurrentLimitConfiguration()
        turnSupplyCurrentConfig.currentLimit = 30.0
        turnMotorConfig.supplyCurrLimit = turnSupplyCurrentConfig
        self.turnMotor.configAllSettings(turnMotorConfig, 50)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Coast)
        
        driveMotorConfig = ctre.TalonFXConfiguration()
        driveMotorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
        driveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero
        driveMotorConfig.slot0.kP = 0.0005
        driveMotorConfig.slot0.kI = 0.0005
        driveMotorConfig.slot0.kD = 0.0000
        driveMotorConfig.slot0.kF = 0.045
        driveMotorConfig.slot0.integralZone = 500
        driveMotorConfig.slot0.allowableClosedloopError = 0
        driveSupplyCurrentConfig = ctre.SupplyCurrentLimitConfiguration()
        driveSupplyCurrentConfig.currentLimit = 37.5
        driveMotorConfig.supplyCurrLimit = driveSupplyCurrentConfig
        self.driveMotor.configAllSettings(driveMotorConfig, 50)
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Coast)
        
        self.reZeroMotors()
        
    def getAbsolutePositionZeroThreeSixty(self):
        return (self.absoluteEncoder.getAbsolutePosition() * 360) - self.absoluteOffset # returns 0-1, multipling by 360 to get degrees
    
    def getWheelAngleRadians(self):
        adjustedRelValDegrees = ((self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio)) * 360) / (self.countsPerRotation * self.turningGearRatio) # zero to 360 scale
        adjustedRelValDegrees -= 180 # adjusting it to be 180/-180
        if adjustedRelValDegrees < -180:
            adjustedRelValDegrees = 360 + adjustedRelValDegrees
        return math.radians(adjustedRelValDegrees) # converting the value to radians and returning it
    
    def getTurnWheelState(self):
        return geometry.Rotation2d(self.getWheelAngleRadians())
        
    def testPeriodic(self):
        ''' Debugging only '''
        adjustedRelValDegrees = ((self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio)) * 360) / (self.countsPerRotation * self.turningGearRatio)
        adjustedRelValDegrees -= 180
        if adjustedRelValDegrees < -180:
            adjustedRelValDegrees = 360 + adjustedRelValDegrees
        SmartDashboard.putNumber(f"{self.moduleName} Relative:", adjustedRelValDegrees)
        SmartDashboard.putNumber(f"{self.moduleName} Absolute:", self.getAbsolutePositionZeroThreeSixty())
        
    def getSwerveModulePosition(self):
        distanceMeters = -self.driveMotor.getSelectedSensorPosition(0) * self.wheelDiameter * math.pi / (self.countsPerRotation * self.drivingGearRatio) # converting the clicks into distance values, in this case, meters
        angle = self.getTurnWheelState()
        return kinematics.SwerveModulePosition(distanceMeters, angle)
    
    def getSwerveModuleState(self):
        velocityMetersPerSecond = (-self.driveMotor.getSelectedSensorVelocity(0) * 10 * self.wheelDiameter * math.pi) / (self.countsPerRotation * self.drivingGearRatio)
        angle = self.getTurnWheelState()
        return kinematics.SwerveModuleState(velocityMetersPerSecond, angle)
    
    def setStateTurnOnly(self, desiredStateAngle):
        #state.angle.rotateBy(geometry.Rotation2d(math.pi)) TODO: test this instead
        if desiredStateAngle > 0:
            desiredStateAngle -= 180
        else:
            desiredStateAngle += 180
        angle = geometry.Rotation2d(math.radians(desiredStateAngle))
        
        desiredAngleDegrees = angle.degrees()
        if desiredAngleDegrees < 0:
            desiredAngleDegrees += 360
        # so we have to convert this back relative to the actual encoder tick amount on the motor encoder, i essentially do a "reverse" remainder to accomplish this
        motorEncoderTickTarget = (self.turnMotor.getSelectedSensorPosition(0) - (self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio))) + desiredAngleDegrees * self.countsPerRotation * self.turningGearRatio / 360
        self.turnMotor.set(ctre.TalonFXControlMode.Position, motorEncoderTickTarget)
    
    def setState(self, state: kinematics.SwerveModuleState):
        state = kinematics.SwerveModuleState.optimize(state, self.getTurnWheelState())
        desiredStateAngle = state.angle.degrees()
        #state.angle.rotateBy(geometry.Rotation2d(math.pi)) TODO: test this instead
        if desiredStateAngle > 0:
            desiredStateAngle -= 180
        else:
            desiredStateAngle += 180
        state.angle = geometry.Rotation2d(math.radians(desiredStateAngle))
        #SmartDashboard.putNumber(f"{self.moduleName} targetState:", state.angle.degrees())
        if abs(state.speed) < self.wheelVelocityThreshold: # we aren't going fast enough to justify actually trying, coast the drive motor
            self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
            self.driveMotor.setNeutralMode(ctre.NeutralMode.Coast)
        else: # drive normally
            velocity = (state.speed * self.drivingGearRatio * self.countsPerRotation) / (self.wheelDiameter * math.pi * 10) # converting from meters per second to ticks per hundred milliseconds
            self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity)
        
        desiredAngleDegrees = state.angle.degrees()
        if desiredAngleDegrees < 0:
            desiredAngleDegrees += 360
        # so we have to convert this back relative to the actual encoder tick amount on the motor encoder, i essentially do a "reverse" remainder to accomplish this
        motorEncoderTickTarget = (self.turnMotor.getSelectedSensorPosition(0) - (self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio))) + desiredAngleDegrees * self.countsPerRotation * self.turningGearRatio / 360
        self.turnMotor.set(ctre.TalonFXControlMode.Position, motorEncoderTickTarget)
        
    def setNeutralMode(self, mode):
        self.driveMotor.setNeutralMode(mode)
        self.turnMotor.setNeutralMode(mode)
    
    def stop(self):
        self.turnMotor.set(ctre.ControlMode.PercentOutput, 0)
        self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
    
    def reZeroMotors(self):
        self.driveMotor.setSelectedSensorPosition(0, 0, 50)
        self.turnMotor.setSelectedSensorPosition(self.getAbsolutePositionZeroThreeSixty() * self.countsPerRotation * self.turningGearRatio / 360, 0, 50)
                
    def xMode(self):
        self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
        self.setNeutralMode(ctre.NeutralMode.Coast)
        self.setStateTurnOnly(self.xAngle)