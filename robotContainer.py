import commands2
from commands2 import button, cmd
import wpilib
import photonvision
from wpimath import controller
import os
import json
from wpimath import geometry, trajectory
from math import radians
import math
import pathplannerlib
from typing import List

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.auxiliaryStreamDeck import AuxiliaryStreamDeckSubsystem

from commands.mandibleCommands import MandibleIntakeCommand, MandibleOuttakeCommand
from commands.substationPickupCommand import SubstationPickupCommand
from commands.placeOnGridCommand import PlaceOnGridCommand

from auto.swerveAutoBuilder import SwerveAutoBuilder

class RobotContainer:
    PLACEHOLDER = geometry.Pose2d()
    
    folderPath = os.path.dirname(os.path.abspath(__file__))
    filePath = os.path.join(folderPath, 'config.json')
    with open (filePath, "r") as f1:
        config = json.load(f1)
    tempAutoList = os.listdir(os.path.join(folderPath, 'deploy/pathplanner'))
    autoList = []
    for pathName in tempAutoList:
        if pathName != "Taxi.path":   
            autoList.append(pathName.removesuffix(".path"))
    print(autoList)
    maxAngularVelocity = config["autonomousSettings"]["autoVelLimit"] / math.hypot(config["RobotDimensions"]["trackWidth"] / 2, config["RobotDimensions"]["wheelBase"] / 2)
    photonCameras = [photonvision.PhotonCamera("photonCameraOne"), photonvision.PhotonCamera("photonCameraOne")]
    
    def __init__(self) -> None:
        # subsystems
        self.driveTrain = DriveTrainSubSystem(self.config)
        self.poseEstimator = PoseEstimatorSubsystem(self.photonCameras, self.driveTrain, geometry.Pose2d(), self.config) # NOTE: THE BLANK POSE2D IS TEMPORARY
        self.mandible = MandibleSubSystem(self.config)
        self.arm = ArmSubSystem(self.config)
        self.auxiliaryStreamDeck = AuxiliaryStreamDeckSubsystem(2)
        
        # buttons
        self.joystick = button.CommandJoystick(0)
        self.xboxController = button.CommandXboxController(1)
        
        #subsystem configuration
        self.driveTrain.setDefaultCommand(cmd.run(lambda: self.driveTrain.joystickDrive(self.getJoystickInput(), self.poseEstimator.getCurrentPose()), [self.driveTrain])) # this is what makes the robot drive, since there isn't a way to bind commands to axes
        self.arm.setDefaultCommand(cmd.run(lambda: self.arm.manualControlIncrementor(self.getStickInput(-self.xboxController.getLeftY(), self.config["ArmConfig"]["manualControlDeadzone"])), [self.arm])) # same issue here, no way to bind commands to axes so this is the solution
        self.mandible.setDefaultCommand(cmd.run(lambda: self.mandible.cubePeriodic(), [self.mandible]))
        
        # additional configuration
        
        # self.LEDS = wpilib.AddressableLED() NOTE: For later
        
        self.generateSimpleAutonomousCommands()
        self.generateAutonomousMarkers()
        
        # this part can be extremely confusing but it is essentially just an initation for the helper class that makes swerve auto possible
        thetaControllerConstraints = [self.config["autonomousSettings"]["rotationPIDConstants"], trajectory.TrapezoidProfileRadians.Constraints(self.config["autonomousSettings"]["autoVelLimit"], self.maxAngularVelocity)]
        self.pathConstraints = pathplannerlib.PathConstraints(self.config["autonomousSettings"]["autoVelLimit"], self.config["autonomousSettings"]["autoAccLimit"])
        self.SwerveAutoBuilder = SwerveAutoBuilder(self.poseEstimator, 
                                                   self.driveTrain, 
                                                   self.eventMap, 
                                                   True, 
                                                   self.config["autonomousSettings"]["translationPIDConstants"],
                                                   thetaControllerConstraints,
                                                   geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], 
                                                                                          y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), 
                                                                                          geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"]
                                                                                                                      )
                                                                                                              )
                                                                                          )
                                                   )
        
        self.configureButtonBindings()
        
        self.gamePieceChooser = wpilib.SendableChooser()
        self.gamePieceChooser.setDefaultOption("Cube", "kCube")
        self.gamePieceChooser.addOption("Cone", "kCone")
        wpilib.SmartDashboard.putData("Game Piece Chooser", self.gamePieceChooser)
        
        ''' Auto-populating our autonomous chooser with the paths we have in the deploy/pathplanner folder '''
        self.autonomousChooser = wpilib.SendableChooser()
        self.autonomousChooser.setDefaultOption("Taxi", "Taxi")
        for pathName in self.autoList:
            self.autonomousChooser.addOption(pathName, pathName)
        wpilib.SmartDashboard.putData("Autonomous Chooser", self.autonomousChooser)
        
    
    def configureButtonBindings(self):
        self.joystickButtonFour = button.JoystickButton(self.joystick, 4)
        self.joystickButtonFour.whileHeld(cmd.run(lambda: self.driveTrain.xMode(), [self.driveTrain]))
        
        self.joystickButtonTwo = button.JoystickButton(self.joystick, 2) # using this as a universal button for macros
        self.joystickButtonTwo.whileHeld(PlaceOnGridCommand(self.SwerveAutoBuilder, 
                                                            self.mandible, 
                                                            self.arm, 
                                                            self.poseEstimator, 
                                                            self.pathConstraints, 
                                                            self.eventMap, 
                                                            self.joystick, 
                                                            self.auxiliaryStreamDeck))
        
        self.joystickButtonThree = button.JoystickButton(self.joystick, 3)
        self.joystickButtonThree.whileHeld(SubstationPickupCommand(self.SwerveAutoBuilder, 
                                                                   self.mandible, 
                                                                   self.arm, 
                                                                   self.poseEstimator, 
                                                                   self.pathConstraints, 
                                                                   self.eventMap, 
                                                                   self.joystick))
        self.xboxController.A().whileTrue(MandibleIntakeCommand(self.mandible))
        self.xboxController.Y().whileTrue(MandibleOuttakeCommand(self.mandible))
        self.xboxController.X().onTrue(self.eventMap["CloseMandible"])
        self.xboxController.B().onTrue(self.eventMap["OpenMandible"])
        
        self.joystickButtonTwelve = button.JoystickButton(self.joystick, 12)
        self.joystickButtonTwelve.onTrue(cmd.run(lambda: self.arm.setPosition("highCone"), [self.arm]))
        
        self.joystickButtonSeven = button.JoystickButton(self.joystick, 7)
        self.joystickButtonSeven.onTrue(cmd.run(lambda: self.arm.setPosition("fullyRetracted"), [self.arm]))
    
    def getStickInput(self, input: float, threshold: float) -> float:
        if abs(input) > threshold:
            adjustedValue = (abs(input) - threshold) / (1 - threshold)
            if input < 0 and adjustedValue != 0:
                adjustedValue = -adjustedValue
        else:
            adjustedValue = 0
        return adjustedValue
        
    def getJoystickInput(self):
        inputs = (self.joystick.getX(), self.joystick.getY(), self.joystick.getZ())
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = self.config["driverStation"]["joystickDeadZones"][(list(self.config["driverStation"]["joystickDeadZones"])[idx])]
            if abs(input) > threshold: 
                adjustedValue = (abs(input) - threshold) / (1 - threshold)
                if input < 0 and adjustedValue != 0:
                    adjustedValue = -adjustedValue
            else:
                adjustedValue = 0
            adjustedInputs.append(adjustedValue)
        return adjustedInputs[0], adjustedInputs[1], adjustedInputs[2]
    
    def evaluateDeadzones(self, inputs):
        '''This method takes in a list consisting of x input, y input, z input
        The magnitude of the units has to be less than 1.
        Returns the list of inputs with zero in place of values less than their respective deadzones.'''
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = self.config["driverStation"]["joystickDeadZones"][(list(self.config["driverStation"]["joystickDeadZones"])[idx])]
            if abs(input) > threshold: 
                adjustedValue = (abs(input) - threshold) / (1 - threshold)
                if input < 0 and adjustedValue != 0:
                    adjustedValue = -adjustedValue
            else:
                adjustedValue = 0
            adjustedInputs.append(adjustedValue)
        return adjustedInputs
    
    def getAutonomousCommand(self):
        return commands2.SequentialCommandGroup(
            [
                #PlaceOnGridCommand(self.SwerveAutoBuilder, self.mandible, self.arm, self.poseEstimator, self.pathConstraints, self.eventMap, self.joystick, self.auxiliaryStreamDeck),
                self.SwerveAutoBuilder.fullAuto(self.getPathGroup(self.selectedAutoName))
            ]
        )
    
    def getPath(self, pathName: str) -> pathplannerlib.PathPlannerTrajectory:
        return pathplannerlib.PathPlanner.loadPath(pathName, self.pathConstraints, False)
    
    def getPathGroup(self, groupName: str) -> List[pathplannerlib.PathPlannerTrajectory]:
        return pathplannerlib.PathPlanner.loadPathGroup(groupName, [self.pathConstraints], False)
    
    def teleopInit(self):
        self.driveTrain.navx.reset()
    
    def disabledInit(self):
        self.driveTrain.coast()
    
    def generateAutonomousMarkers(self):
        self.eventMap = {
            "OpenMandible": cmd.runOnce(lambda: self.mandible.setState('cube'), [self.mandible]),
            "CloseMandible": cmd.runOnce(lambda: self.mandible.setState("cone"), [self.mandible]),
            "OuttakeMandible": self.MandibleOuttakeCommand,
            "IntakeMandible": MandibleIntakeCommand(self.mandible),
            "armFullyRetracted": cmd.runOnce(lambda: self.arm.setPosition("fullyRetracted"), [self.arm]),
            "armSubstation": cmd.runOnce(lambda: self.arm.setPosition("substation"), [self.arm]),
            "armFloor": cmd.runOnce(lambda: self.arm.setPosition("floor"), [self.arm]),
            "armHighCone": cmd.runOnce(lambda: self.arm.setPosition("highCone"), [self.arm]),
            "armMidCone": cmd.runOnce(lambda: self.arm.setPosition("midCone"), [self.arm]),
            "armHighCube": cmd.runOnce(lambda: self.arm.setPosition("highCube"), [self.arm]),
            "armMidCube": cmd.runOnce(lambda: self.arm.setPosition("midCube"), [self.arm])
        }
    
    def generateSimpleAutonomousCommands(self):
        ''' Generates some of the autonomous commands that use *less* logic for use in the auto event map and for manual controls '''
        self.MandibleOuttakeCommand = commands2.SequentialCommandGroup(
            [
                cmd.runOnce(lambda: self.mandible.outtake(), [self.mandible]), 
                commands2.WaitCommand(0.5), # TODO: Test this 0.5 second figure, maybe make this a config variable in the future?
                cmd.runOnce(lambda: self.mandible.coast(), [self.mandible])
            ]
        )
    
    def testPeriodic(self):
        limitSwitchPressed = not wpilib.DigitalInput(0).get()
        wpilib.SmartDashboard.putBoolean("Limit Switch", limitSwitchPressed)
        
    def disabledPeriodic(self):
        wpilib.SmartDashboard.putNumber("Front Left", self.driveTrain.frontLeft.getAbsolutePositionZeroThreeSixty())
        wpilib.SmartDashboard.putNumber("Front Right", self.driveTrain.frontRight.getAbsolutePositionZeroThreeSixty())
        wpilib.SmartDashboard.putNumber("Rear Right", self.driveTrain.rearRight.getAbsolutePositionZeroThreeSixty())
        wpilib.SmartDashboard.putNumber("Rear Left", self.driveTrain.rearLeft.getAbsolutePositionZeroThreeSixty())