import commands2
import math
import pathplannerlib
import wpilib

from commands2 import cmd, button
from wpimath import geometry, kinematics, controller
from typing import List

from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.streamDeck import StreamDeckSubsystem

from commands.armConfirmPlacementCommand import ArmConfirmPlacementCommand
from commands.mandibleCommands import MandibleOuttakeCommand
from networktables import NetworkTable


class AutoGridPlacer:
    placementSequenceCounter = 0
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, 
                 Mandible: MandibleSubSystem, 
                 Arm: ArmSubSystem, 
                 PoseEstimator: PoseEstimatorSubsystem, 
                 DriveTrain: DriveTrainSubSystem, 
                 Constraints: pathplannerlib.PathConstraints, 
                 DriverJoystick: button.CommandJoystick, 
                 StreamDeck: StreamDeckSubsystem, 
                 config: dict, 
                 LimelightTable: NetworkTable,
                 PlacementSpots: List[wpilib.SendableChooser]) -> None:
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.driveTrain = DriveTrain
        self.constraints = Constraints
        self.joystick = DriverJoystick
        self.streamDeck = StreamDeck
        self.config = config
        self.placementSpots = PlacementSpots
        self.stageController = StageController
        self.llTable = LimelightTable
    
    def reset(self) -> None:
        self.placementSequenceCounter = 0

    def getCommandSequence(self, isAutonomous = False) -> commands2.Command:
        if isAutonomous:
            pass
        else:
            return commands2.SequentialCommandGroup(self.stageController.calculateInitialPPCommand(), LimeLightSanityCheckTwo(self.llTable, self.driveTrain, self.poseEstimator), self.stageController.calculateFinalPPCommand(), DriverConfirmCommand(), self.stageController.calculateFinalArmPosition())

class StageController:
    def __init__(self) -> None:
        pass
    
    def calculateArmPosition(self) -> commands2.Command:
        pass
    
    def calculateFinalArmPosition(self) -> commands2.Command:
        pass
    
    def calculateInitialPPCommand(self) -> commands2.Command:
        pass
    
    def calculateFinalPPCommand(self) -> commands2.Command:
        pass
    

class LimeLightSanityCheckTwo(commands2.CommandBase):
    tolerance = 1 # +/- 1 degrees
    
    def __init__(self, LLTable: NetworkTable, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.llTable = LLTable
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.txController = controller.PIDController(0.1, 0, 0, 0.05)
        self.txController.setTolerance(self.tolerance)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.timeoutCounter = wpilib.Timer()
        
    def initialize(self) -> None:
        self.llTable.putNumber('pipeline', 1) # pipeline index one will be our reflective tape pipeline
        self.finished = False
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.isRedAlliance = True
        else:
            self.isRedAlliance = False
        self.txController.reset()
        self.angleController.reset()
        self.timeoutCounter.reset()
        
    def execute(self) -> None:
        if self.llTable.getNumber('tv', 0) == 1: # does the limelight have any valid targets?
            self.timeoutCounter.stop()
            tx = self.llTable.getNumber('tx', 0) # how far off we are from the target in degrees
            feedback = self.txController.calculate(tx, 0)
            if self.txController.atSetpoint() and self.angleController.atSetpoint():
                self.finished = True
            else:
                currentPose = self.poseEstimator.getCurrentPose()
                if self.isRedAlliance:
                    rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), 0)
                    self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, -feedback, rotationFeedback), self.poseEstimator.getCurrentPose())
                else:
                    rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi)
                    self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, feedback, rotationFeedback), self.poseEstimator.getCurrentPose())
        else:
            self.timeoutCounter.start()
            if self.timeoutCounter.hasElapsed(2):
                self.finished = True
    
    def end(self, interrupted: bool) -> None:
        self.llTable.putNumber('pipeline', 0) # sets it back to our apriltag pipeline
        self.driveTrain.stationary()
    
    def isFinished(self) -> bool:
        return super().isFinished()

class DriverConfirmCommand(commands2.CommandBase):
    
    def __init__(self, Joystick: button.CommandJoystick, DriveTrain: DriveTrainSubSystem) -> None:
        super().__init__()
        self.joystick = Joystick
        self.driveTrain = DriveTrain
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        return super().execute()
    
    def isFinished(self) -> bool:
        if self.joystick.getRawButtonPressed(1):
            return True
        else:
            return False
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()