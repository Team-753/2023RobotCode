from wpimath import geometry, kinematics
import wpilib
import navx
from subsystems.swerveModule import SwerveModule
from ctre import NeutralMode
from wpilib import DriverStation
from wpimath import controller, trajectory
from math import hypot, radians, pi, atan2
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
            
            
        self.navx = navx.AHRS.create_spi()
        
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
        self.longitudinalPID = controller.PIDController(teleopConstants["translationPIDConstants"]["kP"], teleopConstants["translationPIDConstants"]["kI"], teleopConstants["translationPIDConstants"]["kD"], teleopConstants["translationPIDConstants"]["period"])
        self.latitudePID = controller.PIDController(teleopConstants["translationPIDConstants"]["kP"], teleopConstants["translationPIDConstants"]["kI"], teleopConstants["translationPIDConstants"]["kD"], teleopConstants["translationPIDConstants"]["period"])
        
        self.maxAngularVelocity = teleopConstants["teleopVelLimit"] / hypot(self.config["RobotDimensions"]["trackWidth"] / 2, self.config["RobotDimensions"]["wheelBase"] / 2) # about 11 rads per second
        rotationConstants = self.config["autonomousSettings"]["rotationPIDConstants"]
        self.rotationPID = controller.PIDController(rotationConstants["kP"], 0.05, rotationConstants["kD"], rotationConstants["period"])
        self.rotationPID.enableContinuousInput(-pi, pi)
        
        self.poseTolerance = geometry.Pose2d(geometry.Translation2d(x=teleopConstants["xPoseToleranceMeters"], 
                                                                                          y=teleopConstants["yPoseToleranceMeters"]), 
                                                                                          geometry.Rotation2d(radians(teleopConstants["thetaPoseToleranceDegrees"])))
        self.targetPose = geometry.Pose2d()
        self.teleopInitiated = False
        
        self.currentSpeed = 0
        self.currentHeading = geometry.Rotation2d()
        self.alliance = DriverStation.Alliance.kBlue # default alliance
        
    def getNAVXRotation2d(self):
        ''' Returns the robot rotation as a Rotation2d object and offsets by a given amount'''
        return self.navx.getRotation2d()
    
    def autoDrive(self, chassisSpeeds: kinematics.ChassisSpeeds, currentPose: geometry.Pose2d, fieldRelative = True):
        if chassisSpeeds == kinematics.ChassisSpeeds(0, 0, 0):
            self.stationary()
            self.currentSpeed = 0
        else:
            chassisSpeeds.omega = -chassisSpeeds.omega
            self.currentSpeed = hypot(chassisSpeeds.vx, chassisSpeeds.vy)
            wpilib.SmartDashboard.putNumber("targetVX", chassisSpeeds.vx)
            wpilib.SmartDashboard.putNumber("targetVY", chassisSpeeds.vy)
            wpilib.SmartDashboard.putNumber("targetVZ", chassisSpeeds.omega)
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega, currentPose.rotation())) # invert vx and omega
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
            
            self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxAutoSpeed)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
    
    def joystickDrive(self, inputs: list, currentPose: geometry.Pose2d):
        '''
        Drives the robot in teleop using PIDs to assist in keeping the robot where the operator wants to go.
        - Locks rotation to the last angle when over the twist threshold
        - Individually locks translation axes to their last relative postion given joystick use of said axis
        - Possible issues:
            - PID's are janky, and need to be tested
            - Foreseeable issues with extremely quick motions and possible switching of directions.
            - To fix the latter, maybe implement some way where we don't set our target pose until the velocity of that axis has reached near-zero
        '''
        ySpeed, xSpeed, zSpeed = inputs[0], inputs[1], inputs[2] # grabbing our inputs and swapping the respective x and y by default
        if self.alliance == DriverStation.Alliance.kRed: # our field oriented controls would be inverted, so lets fix that
            ySpeed = -ySpeed
            xSpeed = -xSpeed
        if xSpeed == 0 and ySpeed == 0 and zSpeed == 0:
            self.stationary()
        else:
            xSpeed *= self.kMaxSpeed
            ySpeed *= self.kMaxSpeed
            zSpeed *= self.maxAngularVelocity
            self.setSwerveStates(xSpeed, ySpeed, zSpeed, currentPose)
    
    def joystickDriveThetaOverride(self, inputs: list, currentPose: geometry.Pose2d, rotationOverride: geometry.Rotation2d) -> None:
        '''
        Drives the robot in teleop using PIDs to assist in keeping the robot where the operator wants to go.
        - Locks rotation to the last angle when over the twist threshold
        - Individually locks translation axes to their last relative postion given joystick use of said axis
        - Possible issues:
            - PID's are janky, and need to be tested
            - Foreseeable issues with extremely quick motions and possible switching of directions.
            - To fix the latter, maybe implement some way where we don't set our target pose until the velocity of that axis has reached near-zero
        '''
        rotationOverridePose = geometry.Pose2d(geometry.Translation2d(), rotationOverride)
        yScalar, xScalar = inputs[0], inputs[1] # grabbing our inputs and swapping the respective x and y by default
        if self.alliance == DriverStation.Alliance.kRed: # to swap inputs for the red alliance persepective change
            yScalar = -yScalar
            xScalar = -xScalar
        poseError = currentPose.relativeTo(rotationOverridePose) # how far are we off from where our axes setpoints were before?
        xSpeed = xScalar * self.kMaxSpeed
        ySpeed = yScalar * self.kMaxSpeed
        angularVelocityFF = (poseError.rotation().radians() / pi) * self.maxAngularVelocity
        wpilib.SmartDashboard.putNumber("AngularVelocityFFOutput", angularVelocityFF)
        if (abs(poseError.rotation().radians()) > self.poseTolerance.rotation().radians()): # we are over the tolerance threshold
            zSpeed = self.rotationPID.calculate(currentPose.rotation().radians(), rotationOverride.radians()) # keep in mind this is not a scalar, this is a speed
        else:
            zSpeed = 0
        if xSpeed == 0 and ySpeed == 0 and zSpeed == 0:
            self.stationary()
        else:
            self.setSwerveStates(xSpeed, ySpeed, angularVelocityFF + zSpeed, currentPose, False)
            
        
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, currentPose: geometry.Pose2d, fieldOrient = True) -> None:
        self.currentSpeed = hypot(xSpeed, ySpeed)
        if fieldOrient:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed), currentPose.rotation()))
        else:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        
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
        ''' Needs to be written '''
    
    def xMode(self):
        ''' Turns the wheels to form an 'X' making the robot immobile without overcoming friction '''  
        self.frontLeft.xMode()
        self.frontRight.xMode()
        self.rearLeft.xMode()
        self.rearRight.xMode()
        
    def getSwerveModulePositions(self):
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
        
    def resetSwerves(self):
        self.frontLeft.reZeroMotors()
        self.frontRight.reZeroMotors()
        self.rearLeft.reZeroMotors()
        self.rearRight.reZeroMotors()