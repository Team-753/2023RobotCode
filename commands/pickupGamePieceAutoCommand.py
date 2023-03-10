import commands2
import pathplannerlib
from wpimath import geometry, kinematics
import wpilib
import math

from typing import List

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.mandible import MandibleSubSystem

from commands.armConfirmPlacementCommand import ArmConfirmPlacementCommand

from auto.swerveAutoBuilder import SwerveAutoBuilder


class PickupGamePieceAutoCommand(commands2.CommandBase):
    FieldWidth = 16.54175
    robotOffset = 1.4 # test this por favor joe
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
        piecePlacement = geometry.Pose2d(geometry.Translation2d(x = targetGamePieceCoordinates[0], y = targetGamePieceCoordinates[1]), geometry.Rotation2d())
        xDiff = currentPose.X() - piecePlacement.X()
        yDiff = currentPose.Y() - piecePlacement.Y()
        robotRotation = geometry.Rotation2d(xDiff, yDiff).rotateBy(geometry.Rotation2d(-math.pi))
        robotPose = piecePlacement.transformBy(geometry.Transform2d(translation = geometry.Translation2d(x = -math.cos(robotRotation.radians()) * self.robotOffset, y = -math.sin(robotRotation.radians()) * self.robotOffset), rotation = robotRotation))
        onLeFlyTJNumeroUno = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(robotPose, kinematics.ChassisSpeeds(0, 0, 0))]))
        onLeFlyTJNumeroDos = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(robotPose, kinematics.ChassisSpeeds(0, 0, 0)), pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, kinematics.ChassisSpeeds(0, 0, 0))]))
        self.command = commands2.SequentialCommandGroup(self.mandible.intake(), ArmConfirmPlacementCommand(self.arm, "Floor"), onLeFlyTJNumeroUno, self.arm.setPosition("Optimized"), onLeFlyTJNumeroDos)
        self.command.schedule()
    
    def execute(self) -> None:
        return super().execute()
    
    def isFinished(self) -> bool:
        return self.command.isFinished()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)