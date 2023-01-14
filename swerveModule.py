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
    
    def __init__(self, config: dict) -> None:
        self.ID = config["driveMotorID"]
        self.driveMotor = ctre.TalonFX(config["driveMotorID"])
        self.turnMotor = ctre.TalonFX(config["turnMotorID"])
        self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 0)
        self.turnMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 0)
        
        self.absolute = ctre.CANCoder(config["encoderID"])
        self.absolute.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absolute.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.absolute.configMagnetOffset(-config["encoderOffset"])
        
        '''self.turnController = controller.ProfiledPIDControllerRadians(self.kPTurn, self.kITurn, self.kDTurn, trajectory.TrapezoidProfileRadians.Constraints(self.maxAngularVelocity, self.maxAngularAcceleration))
        self.turnController.setTolerance(self.angleTolerance)
        self.turnController.enableContinuousInput(-math.pi, math.pi)'''
        self.turnMotor.configIntegratedSensorAbsoluteRange(ctre.AbsoluteSensorRange.Signed_PlusMinus180)
        self.turnMotor.setSelectedSensorPosition(int(self.absolute.getAbsolutePosition() * self.countsPerRotation * self.turningGearRatio / 180), 0)
        print(self.turnMotor.getSelectedSensorPosition(0))
        self.turnMotor.config_kP(0, 0.1)
        self.turnMotor.config_kI(0, 0.0001)
        self.turnMotor.config_kD(0, 0.0001)
        self.turnMotor.config_kF(0, 0)
        self.turnMotor.config_IntegralZone(0, 100)
        self.turnMotor.configAllowableClosedloopError(0, 25)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Brake)
        
        self.driveMotor.config_kP(0, 0.0005)
        self.driveMotor.config_kI(0, 0.0005)
        self.driveMotor.config_kD(0, 0)
        self.driveMotor.config_kF(0, 0.045)
        self.driveMotor.config_IntegralZone(0, 50)
        self.driveMotor.configAllowableClosedloopError(0, 25)
        
        
    def getSwerveModulePosition(self):
        distanceMeters = self.driveMotor.getSelectedSensorPosition(0) * self.wheelDiameter * math.pi / self.countsPerRotation * self.drivingGearRatio # converting the clicks into distance values, in this case, meters
        angle = geometry.Rotation2d(self.getTurnMotorPosition())
        return kinematics.SwerveModulePosition(distanceMeters, angle)
    
    def getTurnMotorPosition(self):
        wheelPositionRadians = ((self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio)) * 2 * math.pi / (self.countsPerRotation * self.turningGearRatio))
        if wheelPositionRadians > math.pi:
            wheelPositionRadians -= 2*math.pi
        return wheelPositionRadians
        
    def setState(self, state):
        #state = kinematics.SwerveModuleState.optimize(state, geometry.Rotation2d(self.getTurnMotorPosition()))
        #velocityMPS = self.driveMotor.getSelectedSensorVelocity(0) * 10 * 0.1016 * math.pi / (self.CPR * 8.14)
        velocity = (state.speed * self.countsPerRotation * self.drivingGearRatio) / (10 * self.wheelDiameter * math.pi) # converting from m/s to ticks/100ms
        targetWheelPositionUnits = state.angle.radians() * self.turningGearRatio * self.countsPerRotation / 2 * math.pi
        #turnOutput = self.turnController.calculate(self.getTurnMotorPosition(), state.angle.radians())
        self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity) #self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity) # assuming ticks/100ms velocity control
        #self.turnMotor.set(ctre.TalonFXControlMode.Position, targetWheelPositionUnits)
        self.turnMotor.set(ctre.TalonFXControlMode.Position, targetWheelPositionUnits)
        print(f"{self.ID}: {self.turnMotor.getSelectedSensorPosition(0)}")
        
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