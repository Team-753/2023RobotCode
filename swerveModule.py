import ctre
from wpimath import controller, trajectory, kinematics, geometry
import math
import wpilib
from ctre import AbsoluteSensorRange, SensorInitializationStrategy

class SwerveModule:
    countsPerRotation = 2048 # integrated sensor counts 2048 units per full rotation
    turningGearRatio = 12.8 # 12.8 motor spins for one rotation
    drivingGearRatio = 8.14 # 8.14 motor spins for one wheel spin
    wheelDiameter = 0.1010 # meters
    kPTurn = 0 # change
    kITurn = 0 # this
    kDTurn = 0 # later
    maxAngularVelocity = math.pi
    maxAngularAcceleration = 2*math.pi
    angleTolerance = 0.00576 # about 1/3 of a degree
    
    def __init__(self, config: dict) -> None:
        self.driveMotor = ctre.TalonFX(config["driveMotorID"])
        self.turnMotor = ctre.TalonFX(config["turnMotorID"])
        self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 0)
        self.turnMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 0)
        
        self.absolute = ctre.CANCoder(config["encoderID"])
        self.absolute.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absolute.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.absolute.configMagnetOffset(config["encoderOffset"])
        
        self.turnController = controller.ProfiledPIDControllerRadians(self.kPTurn, self.kITurn, self.kDTurn, trajectory.TrapezoidProfileRadians.Constraints(self.maxAngularVelocity, self.maxAngularAcceleration))
        self.turnController.setTolerance(self.angleTolerance)
        self.turnController.enableContinuousInput(-math.pi, math.pi)
        
        self.turnMotor.setSelectedSensorPosition(self.absolute.getAbsolutePosition() * self.countsPerRotation * self.turningGearRatio / 180)
        
        
    def getSwerveModulePosition(self):
        distanceMeters = self.driveMotor.getSelectedSensorPosition() * self.wheelDiameter * math.pi / self.countsPerRotation * self.drivingGearRatio # converting the clicks into distance values, in this case, meters
        angle = geometry.Rotation2d(self.getTurnMotorPosition())
        return kinematics.SwerveModulePosition(distanceMeters, angle)
    
    def getTurnMotorPosition(self):
        wheelPositionRadians = ((self.turnMotor.getSelectedSensorPosition(0) % (self.countsPerRotation * self.turningGearRatio)) * 2 * math.pi / (self.countsPerRotation * self.turningGearRatio))
        if wheelPositionRadians > math.pi:
            wheelPositionRadians -= 2*math.pi
        return wheelPositionRadians
        
    def setState(self, desiredState):
        state = kinematics.SwerveModuleState.optimize(desiredState, geometry.Rotation2d(self.getTurnMotorPosition()))
        #velocityMPS = self.driveMotor.getSelectedSensorVelocity(0) * 10 * 0.1016 * math.pi / (self.CPR * 8.14)
        velocity = (state.speed * self.countsPerRotation * self.drivingGearRatio) / (10 * self.wheelDiameter * math.pi) # converting from m/s to ticks/100ms
        turnOutput = self.turnController.calculate(self.getTurnMotorPosition(), state.angle.radians())
        self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity) # assuming ticks/100ms velocity control
        self.turnMotor.set(ctre.TalonFXControlMode.Current, turnOutput)
        
    def setNeutralMode(self, mode):
        self.driveMotor.setNeutralMode(mode)
        self.turnMotor.setNeutralMode(mode)
        
    ''' 
    TODO:
    - Add a well-formatted swerve diagnostics shuffleboard tab
    - Finish X-Mode and Balance functions
    - Integrate Autonomous
    - Refer to: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
    '''