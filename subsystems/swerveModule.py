import ctre
from wpimath import controller, trajectory, kinematics, geometry
import math
import wpilib
from ctre import AbsoluteSensorRange, SensorInitializationStrategy
from wpilib import shuffleboard

class SwerveModule:
    countsPerRotation = 2048 # integrated sensor counts 2048 units per full rotation
    turningGearRatio = 12.8 # 12.8 motor spins for one rotation
    drivingGearRatio = 8.14 # 8.14 motor spins for one wheel spin
    wheelDiameter = 0.1010 # meters
    maxAngularVelocity = math.pi
    maxAngularAcceleration = 2*math.pi
    angleTolerance = 0.00576 # about 1/3 of a degree
    motorEncoderPositionCoefficient = 2 * math.pi / countsPerRotation * 12.8
    
    def __init__(self, config: dict) -> None:
        self.ID = config["driveMotorID"]
        self.driveMotor = ctre.TalonFX(config["driveMotorID"])
        self.turnMotor = ctre.TalonFX(config["turnMotorID"])
        self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 100)
        self.turnMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 100)
        
        self.absolute = ctre.CANCoder(config["encoderID"])
        self.absolute.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 100)
        self.absolute.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, 100)
        self.absolute.configMagnetOffset(-config["encoderOffset"], 100)
        
        #self.turnMotor.configIntegratedSensorAbsoluteRange(ctre.AbsoluteSensorRange.Unsigned_0_to_360, 10)
        self.turnMotor.setSelectedSensorPosition(self.getAbsoluteAngle() / self.motorEncoderPositionCoefficient, 0, 10)
        self.turnMotor.config_kP(0, 0.1)
        self.turnMotor.config_kI(0, 0.0001) #0.0001
        self.turnMotor.config_kD(0, 0.0001)
        self.turnMotor.config_kF(0, 0)
        self.turnMotor.config_IntegralZone(0, 100)
        self.turnMotor.configAllowableClosedloopError(0, 0)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Coast)
        
        self.driveMotor.config_kP(0, 0.0005)
        self.driveMotor.config_kI(0, 0.0005) # 0.0005
        self.driveMotor.config_kD(0, 0)
        self.driveMotor.config_kF(0, 0.045)
        self.driveMotor.config_IntegralZone(0, 50)
        self.driveMotor.configAllowableClosedloopError(0, 25)
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Brake)
        
        
    def getSwerveModulePosition(self):
        distanceMeters = self.driveMotor.getSelectedSensorPosition(0) * self.wheelDiameter * math.pi / self.countsPerRotation * self.drivingGearRatio # converting the clicks into distance values, in this case, meters
        angle = self.getTurnMotorPositionState()
        return kinematics.SwerveModulePosition(distanceMeters, angle)
    
    def getAbsoluteAngle(self):
        angle = self.absolute.getAbsolutePosition()
        angle = math.radians(angle)
        angle %= 2 * math.pi
        if (angle < 0):
            angle += 2 * math.pi
        return angle
    
    def getTurnMotorPositionState(self):
        '''wheelPositionRadians = ((self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio)) * 2 * math.pi / (self.countsPerRotation * self.turningGearRatio))
        if wheelPositionRadians > math.pi:
            wheelPositionRadians -= 2*math.pi'''
        '''motorPosition = ((self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation*self.turningGearRatio)) * 360 /(self.countsPerRotation*self.turningGearRatio))
        if motorPosition > 180:
            motorPosition -= 360
        motorPosition = motorPosition * math.pi / 180'''
        #motorPosition = self.absolute.getAbsolutePosition() * math.pi / 180
        motorPositionRadians = self.turnMotor.getSelectedSensorPosition(0) * self.motorEncoderPositionCoefficient
        motorPositionRadians %= 2 * math.pi
        if (motorPositionRadians < 0):
            motorPositionRadians += 2 * math.pi
        return geometry.Rotation2d(motorPositionRadians)
        
    def setState(self, state):
        #state = kinematics.SwerveModuleState.optimize(state, geometry.Rotation2d(self.getTurnMotorPosition()))
        #velocityMPS = self.driveMotor.getSelectedSensorVelocity(0) * 10 * 0.1016 * math.pi / (self.CPR * 8.14)
        velocity = (state.speed * self.countsPerRotation * self.drivingGearRatio) / (10 * self.wheelDiameter * math.pi) # converting from m/s to ticks/100ms
        self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity) #self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity) # assuming ticks/100ms velocity control
        
        desiredAngleRadians = state.angle.radians()
        currentAngleRadians = self.turnMotor.getSelectedSensorPosition(0) * self.motorEncoderPositionCoefficient
        currentAngleRadiansModulus = currentAngleRadians % 2 * math.pi
        if currentAngleRadiansModulus < 0:
            currentAngleRadiansModulus += 2 * math.pi
        adjustedReferenceAngleRadians = desiredAngleRadians + currentAngleRadians - currentAngleRadiansModulus
        if desiredAngleRadians - currentAngleRadiansModulus > math.pi:
            adjustedReferenceAngleRadians -= 2 * math.pi
        elif desiredAngleRadians - currentAngleRadiansModulus < -math.pi:
            adjustedReferenceAngleRadians += 2 * math.pi
        self.turnMotor.set(ctre.TalonFXControlMode.Position, adjustedReferenceAngleRadians / self.motorEncoderPositionCoefficient)
        if self.ID == 7:
            print(f"target angle motor units: {adjustedReferenceAngleRadians / self.motorEncoderPositionCoefficient}")
            print(f"current angle motor units: {self.turnMotor.getSelectedSensorPosition(0)}")
        
    def setNeutralMode(self, mode):
        self.driveMotor.setNeutralMode(mode)
        self.turnMotor.setNeutralMode(mode)
    
    def resetEncoders(self):
        pass
    ''' 
    TODO:
    - Add a well-formatted swerve diagnostics shuffleboard tab
    - Finish X-Mode and Balance functions
    - Integrate Autonomous
    - Refer to: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
    '''