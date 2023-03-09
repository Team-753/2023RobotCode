import commands2
import pathplannerlib
from wpimath import geometry
import wpilib

from typing import List

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.mandible import MandibleSubSystem

from auto.swerveAutoBuilder import SwerveAutoBuilder


class PickupGamePieceAutoCommand(commands2.CommandBase):
    FieldWidth = 16.54175
    gamePieceXValue = 7.11835 # in meters
    gamePieceYValues = [0.919226, 2.138426, 3.357626, 4.576826]
    verticalStack = [(gamePieceYValues[0] + gamePieceYValues[1]) / 2, (gamePieceYValues[1] + gamePieceYValues[2]) / 2, (gamePieceYValues[2] + gamePieceYValues[3]) / 2]
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Mandible: MandibleSubSystem, SwerveAutoBuilder: SwerveAutoBuilder, Constraints: pathplannerlib.PathConstraints, StartingConfiguration: List[wpilib.SendableChooser]) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.mandible = Mandible
        self.swerveAutoBuilder = SwerveAutoBuilder,
        self.constraints = Constraints
        self.startingConfiguration = StartingConfiguration
    
    def initialize(self) -> None:
        # first step is to geolocate closest game piece.
        # this can be most easily done by splitting the field up into a vertical "stack" per say
        currentPose = self.poseEstimator.getCurrentPose()
        currentYVal = currentPose.Y()
        self.arm.setPosition("Floor")
        if currentYVal <= self.verticalStack[0]:
            targetGamePieceY = self.gamePieceYValues[0] # bottom game piece
            pieceType = self.startingConfiguration[0].getSelected()
        elif currentYVal > self.verticalStack[0] and currentYVal <= self.verticalStack[1]:
            targetGamePieceY = self.gamePieceYValues[1] # 2nd game piece from the bottom
            pieceType = self.startingConfiguration[1].getSelected()
        elif currentYVal > self.verticalStack[1] and currentYVal <= self.verticalStack[2]:
            targetGamePieceY = self.gamePieceYValues[2] # 3rd game piece from the bottom
            pieceType = self.startingConfiguration[2].getSelected()
        else:
            targetGamePieceY = self.gamePieceYValues[3] # 4th game piece from the bottom
            pieceType = self.startingConfiguration[3].getSelected()
        self.mandible.setState(pieceType)
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            targetGamePieceX = self.FieldWidth - self.gamePieceXValue # inverting the target for the red alliance, y is constant
        else:
            targetGamePieceX = self.gamePieceXValue
        targetGamePieceCoordinates = (targetGamePieceX, targetGamePieceY)
    
    def execute(self) -> None:
        return super().execute()
    
    def isFinished(self) -> bool:
        return super().isFinished()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)