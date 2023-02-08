from wpimath import geometry, kinematics
import wpilib
import navx
from subsystems.swerveModule import SwerveModule
from ctre import NeutralMode
from wpilib import shuffleboard
from wpilib import SmartDashboard
import commands2
#from pathplannerlib import 

class DriveTrainSubSystem(commands2.SubsystemBase):
    ''' Swerve drivetrain infrastructure '''
    def __init__(self, MyRobot: commands2.TimedCommandRobot, config: dict) -> None:
        super().__init__()
        self.myRobot = MyRobot
        self.config = config
            
            
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
        
        
    def getNAVXRotation2d(self):
        ''' Returns the robot rotation as a Rotation2d object. '''
        return self.navx.getRotation2d()
    
    def zeroGyro(self):
        self.navx.reset()
    
    def drive(self, chassisSpeeds: kinematics.ChassisSpeeds, fieldRelative = True):
        if chassisSpeeds == kinematics.ChassisSpeeds(0, 0, 0):
            self.coast()
        else:
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, self.getNAVXRotation2d()))
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
            
            self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
    
    def joystickDrive(self, inputs: list, fieldRelative = True):
        xScalar, yScalar, zScalar = inputs[0], inputs[1], inputs[2]
        if xScalar == 0 and yScalar == 0 and zScalar == 0: 
            self.coast()
        else:
            temp = xScalar
            xScalar = yScalar * self.kMaxSpeed
            yScalar = temp * self.kMaxSpeed
            zScalar *= 0.5
            
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(xScalar, yScalar, zScalar), self.getNAVXRotation2d()))
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xScalar, yScalar, zScalar))
            
            self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
        
    def stationary(self):
        ''' Don't use this, is unhealthy for the motors and drivetrain. '''
        self.frontLeft.setNeutralMode(NeutralMode.Brake)
        self.frontLeft.stop()
        self.frontRight.setNeutralMode(NeutralMode.Brake)
        self.frontRight.stop()
        self.rearLeft.setNeutralMode(NeutralMode.Brake)
        self.rearLeft.stop()
        self.rearRight.setNeutralMode(NeutralMode.Brake)
        self.rearRight.stop()
    
    def coast(self):
        ''' Whenever you don't want to power the wheels '''
        self.frontLeft.setNeutralMode(NeutralMode.Coast)
        self.frontLeft.stop()
        self.frontRight.setNeutralMode(NeutralMode.Coast)
        self.frontRight.stop()
        self.rearLeft.setNeutralMode(NeutralMode.Coast)
        self.rearLeft.stop()
        self.rearRight.setNeutralMode(NeutralMode.Coast)
        self.rearRight.stop()
    
    def balance(self):
        ''' Needs a lot of work '''
    
    def xMode(self):
        ''' Needs work '''  
        self.frontLeft.xMode()
        self.frontRight.xMode()
        self.rearLeft.xMode()
        self.rearRight.xMode()
        
    def getSwerveModulePositions(self):
        positions = (self.frontLeft.getSwerveModuleState(), 
               self.frontRight.getSwerveModuleState(), 
               self.rearLeft.getSwerveModuleState(), 
               self.rearRight.getSwerveModuleState())
        return positions
        
    def resetSwerves(self):
        self.frontLeft.reZeroMotors()
        self.frontRight.reZeroMotors()
        self.rearLeft.reZeroMotors()
        self.rearRight.reZeroMotors()