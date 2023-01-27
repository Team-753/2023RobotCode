from wpimath import geometry, kinematics
import wpilib
import navx
from swerveModule import SwerveModule
from ctre import NeutralMode
from wpilib import shuffleboard
from wpilib import SmartDashboard
#from pathplannerlib import 

class DriveTrain:
    ''' Swerve drivetrain infrastructure '''
    dataTab = shuffleboard.Shuffleboard.getTab("Testing")
    def __init__(self, config: dict) -> None:
        self.config = config
        '''self.odometryTab = shuffleboard.Shuffleboard.getTab("Odometry")
        if (config["RobotDefaultSettings"]["DEBUGGING"]):
            self.odometryTab.add("frontLeft", 180).withPosition(0, 0).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(2, 2)
            self.odometryTab.add("frontRight", 45).withPosition(2, 0).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(2, 2)
            self.odometryTab.add("rearLeft", 90).withPosition(0, 2).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(2, 2)
            self.odometryTab.add("rearRight", 135).withPosition(2, 2).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(2, 2)'''
            
            
        self.navx = navx.AHRS.create_spi()
        
        self.kMaxSpeed = config["RobotDefaultSettings"]["robotVelocityLimit"]
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        
        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(self.trackWidth / 2, self.wheelBase / 2),
            geometry.Translation2d(self.trackWidth / 2, -self.wheelBase / 2),
            geometry.Translation2d(-self.trackWidth / 2, self.wheelBase / 2),
            geometry.Translation2d(-self.trackWidth / 2, -self.wheelBase / 2))
        
        self.frontLeft = SwerveModule(self.config["SwerveModules"]["frontLeft"], "frontLeft")
        self.frontRight = SwerveModule(self.config["SwerveModules"]["frontRight"], "frontRight")
        self.rearLeft = SwerveModule(self.config["SwerveModules"]["rearLeft"], "rearLeft")
        self.rearRight = SwerveModule(self.config["SwerveModules"]["rearRight"], "rearRight")
        
        '''self.flVisual = self.dataTab.add("frontLeft", 0).withPosition(0, 0).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(1, 1).getEntry()
        self.frVisual = self.dataTab.add("frontRight", 0).withPosition(2, 0).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(1, 1).getEntry()
        self.rlVisual = self.dataTab.add("rearLeft", 0).withPosition(0, 2).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(1, 1).getEntry()
        self.rrVisual = self.dataTab.add("rearRight", 0).withPosition(2, 2).withWidget(shuffleboard.BuiltInWidgets.kGyro).withSize(1, 1).getEntry()'''
        
    def getNAVXRotation2d(self):
        ''' Returns the robot rotation as a Rotation2d object. '''
        return self.navx.getRotation2d()
    
    def drive(self, xSpeed: float, ySpeed: float, rotation: float, fieldRelative: bool):
        if fieldRelative:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(xSpeed, ySpeed, rotation), self.getNAVXRotation2d()))
        else:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, rotation))
        
        self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
        self.frontLeft.setState(swerveModuleStates[0])
        self.frontRight.setState(swerveModuleStates[1])
        self.rearLeft.setState(swerveModuleStates[2])
        self.rearRight.setState(swerveModuleStates[3])
    
    def joystickDrive(self, xScalar: float, yScalar: float, zScalar: float, fieldRelative: bool):
        temp = xScalar
        xScalar = yScalar * self.kMaxSpeed
        yScalar = temp * self.kMaxSpeed
        zScalar *= 0.1
        
        if fieldRelative:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(xScalar, yScalar, zScalar), self.getNAVXRotation2d()))
        else:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xScalar, yScalar, zScalar))
        
        self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
        self.frontLeft.setState(swerveModuleStates[0])
        self.frontRight.setState(swerveModuleStates[1])
        self.rearLeft.setState(swerveModuleStates[2])
        self.rearRight.setState(swerveModuleStates[3])
        
        
    def followTrajectory(self, trajectory):
        pass
        
    def stationary(self):
        '''self.frontLeft.setNeutralMode(NeutralMode.Brake)
        self.frontRight.setNeutralMode(NeutralMode.Brake)
        self.rearLeft.setNeutralMode(NeutralMode.Brake)
        self.rearRight.setNeutralMode(NeutralMode.Brake)'''
    
    def coast(self):
        self.frontLeft.setNeutralMode(NeutralMode.Coast)
        self.frontRight.setNeutralMode(NeutralMode.Coast)
        self.rearLeft.setNeutralMode(NeutralMode.Coast)
        self.rearRight.setNeutralMode(NeutralMode.Coast)
    
    def balance(self):
        ''' Needs a lot of work '''
    
    def xMode(self):
        ''' Needs work '''
        
    '''def getSwerveModulePositions(self):
        positions = (self.frontLeft.getSwerveModulePosition(), 
               self.frontRight.getSwerveModulePosition(), 
               self.rearLeft.getSwerveModulePosition(), 
               self.rearRight.getSwerveModulePosition())
        return positions'''
    def reportSwerves(self):
        self.frontLeft.testPeriodic()
        self.frontRight.testPeriodic()
        self.rearLeft.testPeriodic()
        self.rearRight.testPeriodic()