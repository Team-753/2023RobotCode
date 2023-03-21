from wpimath import geometry, kinematics
import wpilib
import navx
from subsystems.swerveModule import SwerveModule
from ctre import NeutralMode
from wpilib import DriverStation
from wpimath import controller, trajectory
from math import hypot, radians, pi, atan2
from typing import List
import commands2
#from pathplannerlib import 

class DriveTrainSubSystem(commands2.SubsystemBase):
    ''' Swerve drivetrain infrastructure '''
    '''
    NOTE:
    2023 Robot Dimensions:
    Trackwidth Meters: 0.4954524
    Wheelbase Meters: 0.5969
    When Zeroing turn gears to the right
    '''
    def __init__(self, config: dict) -> None:
        super().__init__()
        self.config = config
            
        self.navx = navx.AHRS.create_spi(update_rate_hz=100)
        
        self.kMaxSpeed = self.config["RobotDefaultSettings"]["wheelVelocityLimit"]
        self.kMaxAutoSpeed = self.config["autonomousSettings"]["autoVelLimit"]
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
        
        teleopConstants = self.config["driverStation"]["teleoperatedRobotConstants"]
        
        self.maxAngularVelocity = teleopConstants["teleopVelLimit"] / hypot(self.config["RobotDimensions"]["trackWidth"] / 2, self.config["RobotDimensions"]["wheelBase"] / 2) # about 11 rads per second
        rotationConstants = self.config["autonomousSettings"]["rotationPIDConstants"]
        self.rotationPID = controller.PIDController(rotationConstants["kP"], rotationConstants["kI"], rotationConstants["kD"], rotationConstants["period"])
        self.rotationPID.enableContinuousInput(-pi, pi)
        
        self.poseTolerance = geometry.Pose2d(geometry.Translation2d(x=teleopConstants["xPoseToleranceMeters"], 
                                                                                          y=teleopConstants["yPoseToleranceMeters"]), 
                                                                                          geometry.Rotation2d(radians(teleopConstants["thetaPoseToleranceDegrees"])))
        self.speedLimitingFactor = 1
        self.alliance = wpilib.DriverStation.Alliance.kBlue
        
    def getNAVXRotation2d(self) -> geometry.Rotation2d:
        ''' Returns the NAVX rotation represented as a Rotation2d object'''
        return self.navx.getRotation2d()
    
    def autoDrive(self, chassisSpeeds: kinematics.ChassisSpeeds, currentPose: geometry.Pose2d, fieldRelative = True) -> None:
        if chassisSpeeds == kinematics.ChassisSpeeds(0, 0, 0):
            self.stationary()
        else:
            chassisSpeeds.omega = -chassisSpeeds.omega # for whatever reason this has to be inverted otherwise *everything* breaks
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega, currentPose.rotation())) # invert vx and omega
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
            
            self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxAutoSpeed)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
    
    def joystickDrive(self, inputs: list, currentPose: geometry.Pose2d) -> None:
        '''
        Typical teleoperated robot control function, nothing fancy here, just some constants and ratios
        '''
        xSpeed, ySpeed, zSpeed = inputs[0], inputs[1], inputs[2]
        if xSpeed == 0 and ySpeed == 0 and zSpeed == 0:
            self.stationary()
        else:
            xSpeed *= self.speedLimitingFactor
            ySpeed *= self.speedLimitingFactor
            zSpeed *= self.speedLimitingFactor
            self.setSwerveStates(xSpeed, ySpeed, zSpeed, currentPose)
    
    def joystickDriveThetaOverride(self, inputs: list, currentPose: geometry.Pose2d, rotationOverride: geometry.Rotation2d, inverted = False) -> None:
        '''
        Drives the robot oriented towards a given target based on the passed in rotation
        '''
        rotationOverridePose = geometry.Pose2d(geometry.Translation2d(), rotationOverride)
        yScalar, xScalar = inputs[0], inputs[1] # grabbing our inputs and swapping the respective x and y by default
        poseError = currentPose.relativeTo(rotationOverridePose) # how far are we off from where our axes setpoints were before?
        xSpeed = xScalar * self.kMaxSpeed
        ySpeed = yScalar * self.kMaxSpeed
        if inverted:
            ySpeed = -ySpeed
            xSpeed = -xSpeed
        angularVelocityFF = (poseError.rotation().radians() / pi) * self.maxAngularVelocity
        if (abs(poseError.rotation().radians()) > self.poseTolerance.rotation().radians()): # we are over the tolerance threshold
            zSpeed = self.rotationPID.calculate(currentPose.rotation().radians(), rotationOverride.radians()) # keep in mind this is not a scalar, this is a speed
        else:
            zSpeed = 0
        if xSpeed == 0 and ySpeed == 0 and zSpeed == 0:
            self.stationary()
        else:
            self.setSwerveStates(xSpeed, ySpeed, angularVelocityFF + zSpeed, currentPose, False)
            
        
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, currentPose: geometry.Pose2d, fieldOrient = True) -> None:
        if fieldOrient:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed), currentPose.rotation()))
        else:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        
        self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
        self.frontLeft.setState(swerveModuleStates[0])
        self.frontRight.setState(swerveModuleStates[1])
        self.rearLeft.setState(swerveModuleStates[2])
        self.rearRight.setState(swerveModuleStates[3])
    
    def stationary(self) -> None:
        ''' Don't use this, is unhealthy for the motors and drivetrain. '''
        self.frontLeft.setNeutralMode(NeutralMode.Brake)
        self.frontLeft.stop()
        self.frontRight.setNeutralMode(NeutralMode.Brake)
        self.frontRight.stop()
        self.rearLeft.setNeutralMode(NeutralMode.Brake)
        self.rearLeft.stop()
        self.rearRight.setNeutralMode(NeutralMode.Brake)
        self.rearRight.stop()
    
    def coast(self) -> None:
        ''' Whenever you don't want to power the wheels '''
        self.frontLeft.setNeutralMode(NeutralMode.Coast)
        self.frontLeft.stop()
        self.frontRight.setNeutralMode(NeutralMode.Coast)
        self.frontRight.stop()
        self.rearLeft.setNeutralMode(NeutralMode.Coast)
        self.rearLeft.stop()
        self.rearRight.setNeutralMode(NeutralMode.Coast)
        self.rearRight.stop()
    
    def xMode(self) -> None:
        ''' Turns the wheels to form an 'X' making the robot immobile without overcoming friction '''  
        self.frontLeft.xMode()
        self.frontRight.xMode()
        self.rearLeft.xMode()
        self.rearRight.xMode()
        
    def getSwerveModulePositions(self) -> List[kinematics.SwerveModulePosition]:
        positions = (self.frontLeft.getSwerveModulePosition(), 
               self.frontRight.getSwerveModulePosition(), 
               self.rearLeft.getSwerveModulePosition(), 
               self.rearRight.getSwerveModulePosition())
        return positions
    
    def actualChassisSpeeds(self) -> kinematics.ChassisSpeeds:
        states = (self.frontLeft.getSwerveModuleState(), 
               self.frontRight.getSwerveModuleState(), 
               self.rearLeft.getSwerveModuleState(), 
               self.rearRight.getSwerveModuleState())
        return self.KINEMATICS.toChassisSpeeds(states[0], states[1], states[2], states[3])
        
    def resetSwerves(self) -> None:
        self.frontLeft.reZeroMotors()
        self.frontRight.reZeroMotors()
        self.rearLeft.reZeroMotors()
        self.rearRight.reZeroMotors()
    
    def enableSpeedLimiter(self) -> None:
        self.speedLimitingFactor = 0.5
    
    def disableSpeedLimiter(self) -> None:
        self.speedLimitingFactor = 1.0