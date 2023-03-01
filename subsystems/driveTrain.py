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
        
        self.kMaxSpeed = config["RobotDefaultSettings"]["wheelVelocityLimit"]
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
        
        self.maxAngularVelocity = teleopConstants["teleopVelLimit"] / hypot(self.config["RobotDimensions"]["trackWidth"] / 2, self.config["RobotDimensions"]["wheelBase"] / 2)
        self.thetaControllerConstraints = trajectory.TrapezoidProfileRadians.Constraints(teleopConstants["teleopVelLimit"], self.maxAngularVelocity)
        self.rotationPID = controller.ProfiledPIDControllerRadians(teleopConstants["rotationPIDConstants"]["kP"], teleopConstants["rotationPIDConstants"]["kI"], teleopConstants["rotationPIDConstants"]["kD"], self.thetaControllerConstraints, teleopConstants["rotationPIDConstants"]["period"])
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
    
    def zeroGyro(self):
        self.navx.reset()
    
    def drive(self, chassisSpeeds: kinematics.ChassisSpeeds, fieldRelative = True):
        if chassisSpeeds == kinematics.ChassisSpeeds(0, 0, 0):
            self.brake()
            self.currentSpeed = 0
        else:
            self.currentSpeed = hypot(chassisSpeeds.vx, chassisSpeeds.vy)
            wpilib.SmartDashboard.putNumber("targetVX", chassisSpeeds.vx)
            wpilib.SmartDashboard.putNumber("targetVY", chassisSpeeds.vy)
            wpilib.SmartDashboard.putNumber("targetVZ", chassisSpeeds.omega)
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vy, -chassisSpeeds.vx, -chassisSpeeds.omega, self.getNAVXRotation2d()))
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
            
            self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
    
    '''def joystickDrive(self, inputs: list, fieldRelative = True) -> None:
        yScalar, xScalar, zScalar = inputs[0], inputs[1], inputs[2]
        if xScalar == 0 and yScalar == 0 and zScalar == 0: 
            self.coast()
        else:
            yScalar *= self.kMaxSpeed
            xScalar *= self.kMaxSpeed
            zScalar *= 0.5
            
            self.setSwerveStates(xScalar, yScalar, zScalar, fieldRelative)'''
    
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
        if not self.teleopInitiated:
            self.targetPose = currentPose # to prevent our robot from driving off when we enable and don't yet touch the joystick
            self.teleopInitiated = True
        HOLDPOSITION = False # hard coded for testing, currently incompatible with some macro commands
        yScalar, xScalar, zScalar = inputs[0], inputs[1], inputs[2] # grabbing our inputs and swapping the respective x and y by default
        if self.alliance == DriverStation.Alliance.kRed: # our field oriented controls would be inverted, so lets fix that
            yScalar = -yScalar
            xScalar = -xScalar
        poseError = currentPose.relativeTo(self.targetPose) # how far are we off from where our axes setpoints were before?
        if xScalar == 0 and yScalar == 0 and zScalar == 0: # if we have no inputs, don't bother correcting
            if HOLDPOSITION:
                if (abs(poseError.X()) > self.poseTolerance.X()):
                    xSpeed = self.longitudinalPID.calculate(currentPose.X(), self.targetPose.X())
                else:
                    xSpeed = 0
                if (abs(poseError.Y()) > self.poseTolerance.Y()):
                    ySpeed = self.latitudePID.calculate(currentPose.Y(), self.targetPose.Y())
                else:
                    ySpeed = 0
                if (abs(poseError.rotation().radians()) > self.poseTolerance.rotation().radians()):
                    zSpeed = self.rotationPID.calculate(currentPose.rotation().radians(), self.targetPose.rotation().radians())
                else:
                    zSpeed = 0
                self.setSwerveStates(xSpeed, ySpeed, zSpeed, currentPose) # we want to actually slow down in a normal capacity to facilitate finer control
            else:
                self.coast() # for when we just want to "drift" after letting go of the joystick
        else:
            if xScalar == 0: # first we check if 
                if (abs(poseError.X()) > self.poseTolerance.X()): # we are over the tolerance threshold
                    xSpeed = self.longitudinalPID.calculate(currentPose.X(), self.targetPose.X()) # keep in mind this is not a scalar, this is a speed
                else:
                    xSpeed = 0
            else:
                self.targetPose = geometry.Pose2d(geometry.Translation2d(currentPose.X(), self.targetPose.Y()), self.targetPose.rotation())
                xSpeed = xScalar * self.kMaxSpeed
            if yScalar == 0:
                if (abs(poseError.Y()) > self.poseTolerance.Y()): # we are over the tolerance threshold
                    ySpeed = self.latitudePID.calculate(currentPose.Y(), self.targetPose.Y()) # keep in mind this is not a scalar, this is a speed
                else:
                    ySpeed = 0 # no need to correct, leave the value alone
            else:
                self.targetPose = geometry.Pose2d(geometry.Translation2d(self.targetPose.X(), currentPose.Y()), self.targetPose.rotation())
                ySpeed = yScalar * self.kMaxSpeed
            if zScalar == 0:
                wpilib.SmartDashboard.putNumber("target rotation", self.targetPose.rotation())
                if (abs(poseError.rotation().radians()) > self.poseTolerance.rotation().radians()): # we are over the tolerance threshold
                    zSpeed = self.rotationPID.calculate(currentPose.rotation().radians(), self.targetPose.rotation().radians()) # keep in mind this is not a scalar, this is a speed
                else:
                    zSpeed = 0
            else:
                self.targetPose = geometry.Pose2d(geometry.Translation2d(self.targetPose.X(), self.targetPose.Y()), currentPose.rotation())
                zSpeed = zScalar * self.kMaxSpeed
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
        if self.alliance == DriverStation.Alliance.kRed: # i am not sure how necessary this is
            yScalar = -yScalar
            xScalar = -xScalar
        poseError = currentPose.relativeTo(rotationOverridePose) # how far are we off from where our axes setpoints were before?
        xSpeed = xScalar * self.kMaxSpeed
        ySpeed = yScalar * self.kMaxSpeed
        if (abs(poseError.rotation().radians()) > self.poseTolerance.rotation().radians()): # we are over the tolerance threshold
            zSpeed = self.rotationPID.calculate(currentPose.rotation().radians(), rotationOverride.radians()) # keep in mind this is not a scalar, this is a speed
        else:
            zSpeed = 0
        if xSpeed == 0 and ySpeed == 0 and zSpeed == 0:
            self.stationary()
        else:
            self.setSwerveStates(xSpeed, ySpeed, zSpeed, currentPose, False)
            
        
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
    
    def coast(self):
        ''' Whenever you don't want to power the wheels '''
        self.frontLeft.setNeutralMode(NeutralMode.Brake)
        self.frontLeft.stop()
        self.frontRight.setNeutralMode(NeutralMode.Brake)
        self.frontRight.stop()
        self.rearLeft.setNeutralMode(NeutralMode.Brake)
        self.rearLeft.stop()
        self.rearRight.setNeutralMode(NeutralMode.Brake)
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
    
    def actualChassiSpeeds(self) -> kinematics.ChassisSpeeds:
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