import commands2
import math
import pathplannerlib
import wpilib

from commands2 import cmd, button
from wpimath import geometry, kinematics
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
    
    def reset(self) -> None:
        self.placementSequenceCounter = 0

    def getCommandSequence(self, isAutonomous = False) -> commands2.Command:
        if isAutonomous:
            pass
        else:
            pass
    
class LimeLightSanityCheck(commands2.CommandBase):
    
    def __init__(self, LLTable: NetworkTable, DriveTrain: DriveTrainSubSystem) -> None:
        super().__init__()
        self.llTable = LLTable
        self.driveTrain = DriveTrain
        
    def initialize(self) -> None:
        self.llTable.putNumber('pipeline', 1) # pipeline index one will be our reflective tape pipeline
        
    def execute(self) -> None:
        if self.llTable.getNumber('tv', 0) == 1:
            
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()