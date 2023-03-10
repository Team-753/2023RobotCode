import commands2
from commands2 import cmd
import wpilib
from wpimath import geometry, kinematics
import math
import pathplannerlib

from subsystems.arm import ArmSubSystem
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.mandible import MandibleSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

from auto.swerveAutoBuilder import SwerveAutoBuilder

from commands.armConfirmPlacementCommand import ArmConfirmPlacementCommand
from commands.mandibleCommands import MandibleOuttakeCommand

from typing import List


class AutoPlaceOnGridCommand(commands2.CommandBase):
    GridLayout = [
        [
            [0.4191, 4.987417], 
            [0.4191, 4.424426], 
            [0.4191, 3.861435], 
            [0.8509, 4.987417], 
            [0.8509, 4.424426], 
            [0.8509, 3.861435], 
            [1.27635, 4.987417], 
            [1.27635, 4.424426], 
            [1.27635, 3.861435]
            ], 
        [
            [0.4191, 3.311017], 
            [0.4191, 2.748026], 
            [0.4191, 2.185035], 
            [0.8509, 3.311017], 
            [0.8509, 2.748026], 
            [0.8509, 2.185035], 
            [1.27635, 3.311017], 
            [1.27635, 2.748026], 
            [1.27635, 2.185035]
            ], 
        [
            [0.4191, 1.634617], 
            [0.4191, 1.071626], 
            [0.4191, 0.508635], 
            [0.8509, 1.634617], 
            [0.8509, 1.071626], 
            [0.8509, 0.508635], 
            [1.27635, 1.634617], 
            [1.27635, 1.071626], 
            [1.27635, 0.508635]
            ]
        ]
    FieldWidth = 16.54175
    highConeOffset = 1.397
    highCubeOffset = 1.7018
    midConeOffset = 1.45
    midCubeOffset = 1.7018
    lowConeOffset = 1.5494
    lowCubeOffset = 1.5494
    firstGridYCutoff = 1.905
    secondGridYCutoff = 3.5814
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Arm: ArmSubSystem, Drivetrain: DriveTrainSubSystem, Mandible: MandibleSubSystem, PoseEstimator: PoseEstimatorSubsystem, PlacementSpots: List[wpilib.SendableChooser], Stage: int, PathConstraints: pathplannerlib.PathConstraints) -> None:
        super().__init__()
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.arm = Arm
        self.driveTrain = Drivetrain
        self.mandible = Mandible
        self.poseEstimator = PoseEstimator
        self.placementSpots = PlacementSpots
        self.stage = Stage
        self.constraints = PathConstraints
        
    def initialize(self) -> None:
        # first step is to figure out which grid we're nearest to
        self.finished = False
        alliance = wpilib.DriverStation.getAlliance()
        currentPose = self.poseEstimator.getCurrentPose()
        if currentPose.Y() < self.firstGridYCutoff:
            grid = self.GridLayout[2]
        elif currentPose.Y() < self.secondGridYCutoff:
            grid = self.GridLayout[1]
        else:
            grid = self.GridLayout[0]
        slot = self.placementSpots[self.stage].getSelected()
        self.stage += 1
        allianceFactor = 1
        if alliance == wpilib.DriverStation.Alliance.kRed:
            allianceFactor = -1
            if slot > 6: # low grid
                if slot == 9:
                    slot = 7
                elif slot == 7:
                    slot = 9
            elif slot < 4: # high grid
                if slot == 3:
                    slot = 1
                elif slot == 1:
                    slot = 3
            else: # mid grid
                if slot == 6:
                    slot = 4
                elif slot == 4:
                    slot = 6
        slot -= 1
        targetPosition = grid[slot]
        if alliance == wpilib.DriverStation.Alliance.kRed:
            targetPosition[0] = self.FieldWidth - targetPosition[0]
        if slot > 5: # low-row
            offset = self.lowCubeOffset
            placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "BottomPlacement"), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition("Optimized"), [self.arm]))
            self.arm.setPosition("BottomPlacement")
        elif slot < 3: # high-row
            # our only placement option is perpindicular to the grid and right up against it
            # first we have to calculate our required robot position
            if slot != 1: # it's not a cube so it's a cone
                offset = self.highConeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "HighConePrep"), ArmConfirmPlacementCommand(self.arm, "HighConePlacement"), cmd.runOnce(lambda: self.mandible.setState('Cube'), [self.mandible]), cmd.runOnce(lambda: self.arm.setPosition("Optimized"), [self.arm]), commands2.WaitCommand(0.25), cmd.runOnce(lambda: self.mandible.setState('Cone')))
                self.arm.setPosition("HighConePrep")
            else:
                offset = self.highCubeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "HighCube"), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition("Optimized"), [self.arm]))
                self.arm.setPosition("HighCube")
        else: # mid-row
            if slot != 4: # it's a cone
                offset = self.midConeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "MidConePrep"), ArmConfirmPlacementCommand(self.arm, "MidConePlacement"), cmd.runOnce(lambda: self.mandible.setState('Cube'), [self.mandible]), cmd.runOnce(lambda: self.arm.setPosition("Optimized"), [self.arm]), commands2.WaitCommand(0.25), cmd.runOnce(lambda: self.mandible.setState('Cone')))
                self.arm.setPosition("MidConePrep")
            else:
                offset = self.midCubeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "MidCube"), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition("Optimized"), [self.arm]))
                self.arm.setPosition("MidCube")
        if slot < 3: # we won't do a circle
            piecePlacement = geometry.Pose2d(geometry.Translation2d(x = targetPosition[0], y = targetPosition[1]), geometry.Rotation2d(0.5 * math.pi + (allianceFactor * 0.5 * math.pi)))
            robotPose = piecePlacement.transformBy(geometry.Transform2d(translation = geometry.Translation2d(x = -(offset * allianceFactor), y = 0), rotation = geometry.Rotation2d()))
        else: # we can place relative to a circle
            piecePlacement = geometry.Pose2d(geometry.Translation2d(x = targetPosition[0], y = targetPosition[1]), geometry.Rotation2d())
            xDiff = currentPose.X() - piecePlacement.X()
            yDiff = currentPose.Y() - piecePlacement.Y()
            robotRotation = geometry.Rotation2d(xDiff, yDiff).rotateBy(geometry.Rotation2d(-math.pi))
            robotPose = piecePlacement.transformBy(geometry.Transform2d(translation = geometry.Translation2d(x = -math.cos(robotRotation.radians()) * offset, y = -math.sin(robotRotation.radians()) * offset), rotation = robotRotation))
        onLeFlyTJNumeroUno = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(robotPose, kinematics.ChassisSpeeds(0, 0, 0))]))
        onLeFlyTJNumeroDos = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(robotPose, kinematics.ChassisSpeeds(0, 0, 0)), pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, kinematics.ChassisSpeeds(0, 0, 0))]))
        self.command = commands2.SequentialCommandGroup(onLeFlyTJNumeroUno, placementSequence, onLeFlyTJNumeroDos)
        self.command.schedule()
    
    def execute(self) -> None:
        return super().execute()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return self.command.isFinished()