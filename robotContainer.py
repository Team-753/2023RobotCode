'''
TODO:
Add autonomous compatibility with the place on grid command
'''
from networktables import NetworkTables
import commands2
from commands2 import button, cmd
import wpilib
import photonvision
from wpimath import controller
import os
import json
from wpimath import geometry, trajectory, kinematics
from math import radians
import math
import pathplannerlib
from typing import List

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.streamDeck import StreamDeckSubsystem

from commands.mandibleCommands import MandibleIntakeCommand, MandibleOuttakeCommand
from commands.substationPickupCommand import SubstationPickupCommand
from commands.placeOnGridCommand import PlaceOnGridCommand
from commands.defaultDriveCommand import DefaultDriveCommand
from commands.pickupGamePieceAutoCommand import PickupGamePieceAutoCommand
from commands.autoPlaceOnGridCommand import AutoPlaceOnGridCommand

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
    maxAngularVelocity = config["autonomousSettings"]["autoVelLimit"] / math.hypot(config["RobotDimensions"]["trackWidth"] / 2, config["RobotDimensions"]["wheelBase"] / 2)
    photonCameras = [photonvision.PhotonCamera("photoncameraone")] # , photonvision.PhotonCamera("photoncameratwo")
    NetworkTables.initialize()
    LLTable = NetworkTables.getTable("limelight")
    
    def __init__(self) -> None:
        # buttons
        self.joystick = button.CommandJoystick(0)
        self.xboxController = button.CommandXboxController(1)
        self.auxiliaryStreamDeckJoystick = button.CommandJoystick(2)
        
        # subsystems
        self.driveTrain = DriveTrainSubSystem(self.config)
        self.poseEstimator = PoseEstimatorSubsystem(self.photonCameras, self.driveTrain, geometry.Pose2d(), self.config) # NOTE: THE BLANK POSE2D IS TEMPORARY
        self.mandible = MandibleSubSystem(self.config)
        self.arm = ArmSubSystem(self.config)
        self.streamDeckSubsystem = StreamDeckSubsystem(self.arm, self.auxiliaryStreamDeckJoystick)
        
        
        
        #subsystem configuration
        self.arm.setDefaultCommand(cmd.run(lambda: self.arm.manualControlIncrementor(self.getStickInput(self.xboxController.getLeftY(), self.config["ArmConfig"]["manualControlDeadzone"])), [self.arm])) # same issue here, no way to bind commands to axes so this is the solution
        self.mandible.setDefaultCommand(cmd.run(lambda: self.mandible.cubePeriodic(), [self.mandible])) # this may not run in certain modes
        
        # additional configuration
        
        # self.LEDS = wpilib.AddressableLED() NOTE: For later
        
        # this part can be extremely confusing but it is essentially just an initation for the helper class that makes swerve auto possible
        
        self.gamePieceChooser = wpilib.SendableChooser()
        self.gamePieceChooser.setDefaultOption("Cube", "Cube")
        self.gamePieceChooser.addOption("Cone", "Cone")
        wpilib.SmartDashboard.putData("Game Piece Chooser", self.gamePieceChooser)
        
        ''' Auto-populating our autonomous chooser with the paths we have in the deploy/pathplanner folder '''
        self.autonomousChooser = wpilib.SendableChooser()
        self.autonomousChooser.setDefaultOption("Only Place", "Only Place")
        for pathName in self.autoList:
            self.autonomousChooser.addOption(pathName, pathName)
        wpilib.SmartDashboard.putData("Autonomous Chooser", self.autonomousChooser)
        
        self.gamePieceSlotOne = wpilib.SendableChooser()
        self.gamePieceSlotOne.setDefaultOption("Cube", "Cube")
        self.gamePieceSlotOne.addOption("Cone", "Cone")
        
        self.gamePieceSlotTwo = wpilib.SendableChooser()
        self.gamePieceSlotTwo.setDefaultOption("Cube", "Cube")
        self.gamePieceSlotTwo.addOption("Cone", "Cone")
        
        self.gamePieceSlotThree = wpilib.SendableChooser()
        self.gamePieceSlotThree.setDefaultOption("Cube", "Cube")
        self.gamePieceSlotThree.addOption("Cone", "Cone")
        
        self.gamePieceSlotFour = wpilib.SendableChooser()
        self.gamePieceSlotFour.setDefaultOption("Cube", "Cube")
        self.gamePieceSlotFour.addOption("Cone", "Cone")
        
        wpilib.SmartDashboard.putData("Field Piece One", self.gamePieceSlotOne)
        wpilib.SmartDashboard.putData("Field Piece Two", self.gamePieceSlotTwo)
        wpilib.SmartDashboard.putData("Field Piece Three", self.gamePieceSlotThree)
        wpilib.SmartDashboard.putData("Field Piece Four", self.gamePieceSlotFour)
        
        self.firstGamePiecePlacement = wpilib.SendableChooser()
        self.firstGamePiecePlacement.setDefaultOption("High Cube", 2)
        self.firstGamePiecePlacement.addOption("Left High Cone", 1)
        self.firstGamePiecePlacement.addOption("Right High Cone", 3)
        self.firstGamePiecePlacement.addOption("Mid Cube", 5)
        self.firstGamePiecePlacement.addOption("Left Mid Cone", 4)
        self.firstGamePiecePlacement.addOption("Right Mid Cone", 6)
        wpilib.SmartDashboard.putData("First Auto Game Piece Placement", self.firstGamePiecePlacement)
        
        self.secondGamePiecePlacement = wpilib.SendableChooser()
        self.secondGamePiecePlacement.setDefaultOption("Right High Cone", 3)
        self.secondGamePiecePlacement.addOption("Left High Cone", 1)
        self.secondGamePiecePlacement.addOption("High Cube", 2)
        self.secondGamePiecePlacement.addOption("Mid Cube", 5)
        self.secondGamePiecePlacement.addOption("Left Mid Cone", 4)
        self.secondGamePiecePlacement.addOption("Right Mid Cone", 6)
        wpilib.SmartDashboard.putData("Second Auto Game Piece Placement", self.secondGamePiecePlacement)
        
        self.thirdGamePiecePlacement = wpilib.SendableChooser()
        self.thirdGamePiecePlacement.setDefaultOption("Left High Cone", 1)
        self.thirdGamePiecePlacement.addOption("Right High Cone", 3)
        self.thirdGamePiecePlacement.addOption("High Cube", 2)
        self.thirdGamePiecePlacement.addOption("Mid Cube", 5)
        self.thirdGamePiecePlacement.addOption("Left Mid Cone", 4)
        self.thirdGamePiecePlacement.addOption("Right Mid Cone", 6)
        wpilib.SmartDashboard.putData("Third Auto Game Piece Placement", self.thirdGamePiecePlacement)
        
        self.gamePieceSlots = [self.gamePieceSlotOne, self.gamePieceSlotTwo, self.gamePieceSlotThree, self.gamePieceSlotFour]
        self.gamePiecePlacementList = [self.firstGamePiecePlacement, self.secondGamePiecePlacement, self.thirdGamePiecePlacement]
        self.stages = 0
        thetaControllerConstraints = self.config["autonomousSettings"]["rotationPIDConstants"]
        self.pathConstraints = pathplannerlib.PathConstraints(maxVel = self.config["autonomousSettings"]["autoVelLimit"], maxAccel = self.config["autonomousSettings"]["autoAccelLimit"])
        self.bruhMomentoAutoBuilder = SwerveAutoBuilder(self.poseEstimator, 
                                                   self.driveTrain, 
                                                   {}, 
                                                   False, 
                                                   self.config["autonomousSettings"]["translationPIDConstants"],
                                                   thetaControllerConstraints,
                                                   geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], 
                                                                                          y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), 
                                                                                          geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"]
                                                                                                                      )
                                                                                                              )
                                                                                          )
                                                   )
        self.generateAutonomousMarkers()
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
        
    
    def configureButtonBindings(self):
        self.joystickButtonFour = button.JoystickButton(self.joystick, 4)
        self.joystickButtonFour.whileHeld(cmd.run(lambda: self.driveTrain.xMode(), [self.driveTrain]))
        
        self.joystickButtonTwo = button.JoystickButton(self.joystick, 2)
        self.joystickButtonTwo.whenHeld(PlaceOnGridCommand(self.bruhMomentoAutoBuilder, 
                                                            self.mandible, 
                                                            self.arm, 
                                                            self.poseEstimator, 
                                                            self.driveTrain, 
                                                            self.pathConstraints, 
                                                            self.eventMap, 
                                                            self.joystick, 
                                                            self.streamDeckSubsystem, 
                                                            self.config))
        
        self.joystickButtonThree = button.JoystickButton(self.joystick, 3)
        self.joystickButtonThree.whileHeld(SubstationPickupCommand(self.SwerveAutoBuilder, 
                                                                   self.mandible, 
                                                                   self.arm, 
                                                                   self.poseEstimator, 
                                                                   self.pathConstraints, 
                                                                   self.eventMap, 
                                                                   self.joystick))
        self.xboxController.A().whileTrue(MandibleIntakeCommand(self.mandible))
        self.xboxController.Y().whileTrue(cmd.run(lambda: self.mandible.outtake(), [self.mandible]))
        self.xboxController.X().onTrue(self.eventMap["CloseMandible"])
        self.xboxController.B().onTrue(self.eventMap["OpenMandible"])
        
    
    def getStickInput(self, input: float, threshold: float) -> float:
        if abs(input) > threshold:
            adjustedValue = (abs(input) - threshold) / (1 - threshold)
            if input < 0 and adjustedValue != 0:
                adjustedValue = -adjustedValue
        else:
            adjustedValue = 0
        return adjustedValue    
    
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
        pathName = self.autonomousChooser.getSelected()
        if pathName == "Only Place":
            return self.eventMap["PlaceGamePiece"]
        autoPath = self.getPathGroup(pathName)
        # MAJOR TODO: Fix bug, starting pathplanner position would be off if we are on red alliance, best way to fix is to transform stating holonomic state to blue alliance.
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            startingTargetPose = pathplannerlib.PathPlannerTrajectory.transformTrajectoryForAlliance(autoPath[0], wpilib.DriverStation.Alliance.kRed).getInitialHolonomicPose()
            startingTrajectory = pathplannerlib.PathPlannerTrajectory.transformTrajectoryForAlliance(pathplannerlib.PathPlanner.generatePath(self.pathConstraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.poseEstimator.getCurrentPose(), self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(startingTargetPose, kinematics.ChassisSpeeds(0, 0, 0))]), wpilib.DriverStation.Alliance.kBlue)
        else:
            startingTrajectory = pathplannerlib.PathPlanner.generatePath(self.pathConstraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.poseEstimator.getCurrentPose(), self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(autoPath[0].getInitialHolonomicPose(), kinematics.ChassisSpeeds(0, 0, 0))])
        autoPath.insert(0, startingTrajectory)
        return self.SwerveAutoBuilder.fullAuto(autoPath)
    
    def getPath(self, pathName: str) -> pathplannerlib.PathPlannerTrajectory:
        return pathplannerlib.PathPlanner.loadPath(pathName, self.pathConstraints, False)
    
    def getPathGroup(self, groupName: str) -> List[pathplannerlib.PathPlannerTrajectory]:
        return pathplannerlib.PathPlanner.loadPathGroup(groupName, [self.pathConstraints], False)
    
    def generateAutonomousMarkers(self):
        self.eventMap = {
            "OpenMandible": cmd.runOnce(lambda: self.mandible.setState('Cube'), [self.mandible]),
            "CloseMandible": cmd.runOnce(lambda: self.mandible.setState("Cone"), [self.mandible]),
            "OuttakeMandible": MandibleOuttakeCommand(self.mandible),
            "IntakeMandibleToggle": cmd.runOnce(lambda: self.mandible.intake(), [self.mandible]),
            "StopMandible": cmd.runOnce(lambda: self.mandible.stop(), [self.mandible]),
            "ArmFullyRetracted": cmd.runOnce(lambda: self.arm.setPosition("FullyRetracted"), [self.arm]),
            "ArmSubstation": cmd.runOnce(lambda: self.arm.setPosition("Substation"), [self.arm]),
            "ArmFloor": cmd.runOnce(lambda: self.arm.setPosition("Floor"), [self.arm]),
            "ArmHighConePrep": cmd.runOnce(lambda: self.arm.setPosition("HighConePrep"), [self.arm]),
            "ArmMidConePrep": cmd.runOnce(lambda: self.arm.setPosition("MidConePrep"), [self.arm]),
            "ArmHighConePlacement": cmd.runOnce(lambda: self.arm.setPosition("HighConePlacement"), [self.arm]),
            "ArmMidConePlacement": cmd.runOnce(lambda: self.arm.setPosition("MidConePlacement"), [self.arm]),
            "ArmHighCube": cmd.runOnce(lambda: self.arm.setPosition("HighCube"), [self.arm]),
            "ArmMidCube": cmd.runOnce(lambda: self.arm.setPosition("MidCube"), [self.arm]),
            "ArmOptimized": cmd.runOnce(lambda: self.arm.setPosition("Optimized"), [self.arm]),
            "PlaceGamePiece": AutoPlaceOnGridCommand(self.bruhMomentoAutoBuilder,
                                                     self.arm,
                                                     self.driveTrain,
                                                     self.mandible,
                                                     self.poseEstimator,
                                                     self.gamePiecePlacementList,
                                                     self.stages,
                                                     self.pathConstraints),
            "AutoPiecePickup": PickupGamePieceAutoCommand(self.driveTrain, 
                                                          self.arm, 
                                                          self.poseEstimator, 
                                                          self.mandible, 
                                                          self.bruhMomentoAutoBuilder,
                                                          self.pathConstraints,
                                                          self.gamePieceSlots
                                                          )
        }
    
    def testInit(self):
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.joystick, self.driveTrain, self.poseEstimator, self.config))
        self.arm.setPosition("FullyRetracted")
        self.mandible.setState("Cube") # we need to always be in cube mode or else we can't fit in starting config
    
    def testPeriodic(self):
        pass
        
    def disabledInit(self):
        self.driveTrain.coast()
        self.poseEstimator.isDisabled = True
    
    def disabledPeriodic(self):
        pass
    
    def disabledExit(self):
        self.poseEstimator.isDisabled = False
    
    def autonomousInit(self):
        self.stages = 0
        self.driveTrain.setDefaultCommand(cmd.run(lambda: None, [self.driveTrain])) # stupid ass command based POS
        self.arm.setPosition("Optimized")
        self.mandible.setState(self.gamePieceChooser.getSelected())
    
    def autonomousPeriodic(self):
        pass
        
    def teleopInit(self):
        self.driveTrain.alliance = wpilib.DriverStation.getAlliance()
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.joystick, self.driveTrain, self.poseEstimator, self.config)) # this is what makes the robot drive, since there isn't a way to bind commands to axes
    
    def teleopPeriodic(self):
        pass
        