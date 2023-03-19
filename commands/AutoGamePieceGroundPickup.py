import commands2
import math
import pathplannerlib
import wpilib
import photonvision

from commands2 import cmd, button
from wpimath import geometry, kinematics, controller
from typing import List, Tuple

from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.streamDeck import StreamDeckSubsystem
from commands.turnToCommand import TurnToCommand

from commands.armConfirmPlacementCommand import ArmConfirmPlacementCommand
from commands.mandibleCommands import MandibleOuttakeCommand
from networktables import NetworkTable

class AutoGamePiecePickupController:
    ''' Controls the stages of the autonomous game piece pickup '''
    FieldWidth = 16.54175
    FieldHeight = 8.0137
    gamePieceXValue = 7.11835 # in meters
    gamePieceYValues = [0.919226, 2.138426, 3.357626, 4.576826]
    verticalStack = [(gamePieceYValues[0] + gamePieceYValues[1]) / 2, (gamePieceYValues[1] + gamePieceYValues[2]) / 2, (gamePieceYValues[2] + gamePieceYValues[3]) / 2]
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Mandible: MandibleSubSystem, SwerveAutoBuilder: SwerveAutoBuilder, Constraints: pathplannerlib.PathConstraints, StartingConfiguration: List[wpilib.SendableChooser], LLTable: NetworkTable) -> None:
        self.driveTrain = DriveTrain
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.mandible = Mandible
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.constraints = Constraints
        self.startingConfig = StartingConfiguration
        self.LLtable == LLTable
    
    def reset(self):
        pass
    
    def getCommandSequence(self):
        pass
    
    def calculateNearestPiece(self, currentPose: geometry.Pose2d) -> Tuple(float, str):
        currentYVal = currentPose.Y()
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            transformedStartingConfiguration = self.startingConfig.copy()
            transformedStartingConfiguration.reverse()
            if currentYVal <= self.transformToRedAllianceYVal(self.verticalStack[0]):
                targetGamePieceY = self.transformToRedAllianceYVal(self.GamePieceYValues[0]) # bottom game piece
                pieceType = transformedStartingConfiguration[0].getSelected()
            elif currentYVal > self.transformToRedAllianceYVal(self.verticalStack[0]) and currentYVal <= self.transformToRedAllianceYVal(self.verticalStack[1]):
                targetGamePieceY = self.transformToRedAllianceYVal(self.GamePieceYValues[1]) # 2nd game piece from the bottom
                pieceType = transformedStartingConfiguration[1].getSelected()
            elif currentYVal > self.transformToRedAllianceYVal(self.verticalStack[1]) and currentYVal <= self.transformToRedAllianceYVal(self.verticalStack[2]):
                targetGamePieceY = self.transformToRedAllianceYVal(self.GamePieceYValues[2]) # 3rd game piece from the bottom
                pieceType = transformedStartingConfiguration[2].getSelected()
            else:
                targetGamePieceY = self.transformToRedAllianceYVal(self.GamePieceYValues[3]) # 4th game piece from the bottom
                pieceType = transformedStartingConfiguration[3].getSelected()
                
    def transformToRedAllianceYVal(self, value: float):
        return self.FieldHeight - value
        
        