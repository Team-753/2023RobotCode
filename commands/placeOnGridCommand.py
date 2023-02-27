import pathplannerlib
import commands2
from commands2 import cmd, button
from wpimath import geometry
import wpilib
import math
from typing import List, Tuple

# For intellisense
from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from commands.armConfirmPlacementCommand import ArmConfirmPlacementCommand
from commands.mandibleCommands import MandibleIntakeCommand, MandibleOuttakeCommand

class PlaceOnGridCommand(commands2.CommandBase):
    GridLayout =  [
        [
            [
                1.27635,
                3.861435
            ],
            [
                1.27635,
                4.424426
            ],
            [
                1.27635,
                4.987417
            ],
            [
                0.8509,
                3.861435
            ],
            [
                0.8509,
                4.424426
            ],
            [
                0.8509,
                4.987417
            ],
            [
                0.4191,
                3.861435
            ],
            [
                0.4191,
                4.424426
            ],
            [
                0.4191,
                4.987417
            ]
        ],
        [
            [
                1.27635,
                2.185035
            ],
            [
                1.27635,
                2.748026
            ],
            [
                1.27635,
                3.311017
            ],
            [
                0.8509,
                2.185035
            ],
            [
                0.8509,
                2.748026
            ],
            [
                0.8509,
                3.311017
            ],
            [
                0.4191,
                2.185035
            ],
            [
                0.4191,
                2.748026
            ],
            [
                0.4191,
                3.311017
            ]
        ],
        [
            [
                1.27635,
                0.508635
            ],
            [
                1.27635,
                1.071626
            ],
            [
                1.27635,
                1.634617
            ],
            [
                0.8509,
                0.508635
            ],
            [
                0.8509,
                1.071626
            ],
            [
                0.8509,
                1.634617
            ],
            [
                0.4191,
                0.508635
            ],
            [
                0.4191,
                1.071626
            ],
            [
                0.4191,
                1.634617
            ]
        ]
    ]
    FieldWidth = 16.54175
    highConeOffset = 1.88595
    highCubeOffset = 2 # this
    midConeOffset = 2 # this
    midCubeOffset = 2.15 # this
    lowConeOffset = 1.5 # this
    lowCubeOffset = 1.65 # and this are all total guesses, check them please
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, DriveTrain: DriveTrainSubSystem, Constraints: pathplannerlib.PathConstraints, EventMap: dict, DriverJoystick: button.CommandJoystick, isAutonomous: bool, targetGridSlot: list, config: dict) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm, DriveTrain])
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.eventMap = EventMap
        self.joystick = DriverJoystick
        self.constraints = Constraints
        self.driveTrain = DriveTrain
        self.isAutonomous = isAutonomous
        self.targetGridSlot = targetGridSlot
        self.config = config
        
        
    def initialize(self) -> None:
        self.myTimeHasCome = False
        self.currentPose = self.poseEstimator.getCurrentPose()
        self.onlySetArmPosition(self.targetGridSlot[1])
        if self.isAutonomous:
            self.sequence = self.getTargetGridValues(self.targetGridSlot[0], self.targetGridSlot[1])
            self.swerveAutoBuilder.useAllianceColor = False
            onLeFlyTJ = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.currentPose, self.driveTrain.actualChassiSpeeds()), pathplannerlib.PathPoint(self.sequence[0].translation(), geometry.Rotation2d(), self.sequence[0].rotation())]))
            commands2.SequentialCommandGroup(onLeFlyTJ, self.sequence[1], cmd.runOnce(lambda: self.finished(), []))
            
    def finished(self):
        self.myTimeHasCome = True
    
    def execute(self) -> None:
        if not self.isAutonomous: # well shit it ain't autonomous that's for sure
            self.currentPose = self.poseEstimator.getCurrentPose()
            if self.joystick.getRawButton(1): # is the driver pulling the trigger (confirming game piece placement)
                self.sequence = self.getTargetGridValues(self.targetGridSlot[0], self.targetGridSlot[1])
                self.swerveAutoBuilder.useAllianceColor = False
                onLeFlyTJ = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.currentPose, self.driveTrain.actualChassiSpeeds()), pathplannerlib.PathPoint(self.sequence[0].translation(), geometry.Rotation2d(), self.sequence[0].rotation())]))
                commands2.SequentialCommandGroup(onLeFlyTJ, self.sequence[1], cmd.runOnce(lambda: self.finished(), []))
            else: # they are just holding the side button
                targetRotation = self.calculateRobotAngle(self.currentPose)
                inputs = (self.joystick.getX(), self.joystick.getY())
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
                self.driveTrain.joystickDriveThetaOverride(adjustedInputs, self.currentPose, targetRotation)
    
    def isFinished(self) -> bool:
        return self.myTimeHasCome
    
    def getTargetGridValues(self, grid: int, slot: int) -> Tuple[geometry.Pose2d, commands2.Command]:
        targetCoordinates = self.GridLayout[grid][slot]
        allianceFactor = 1
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed: # we have to flip the x-coordinate if we are on red alliance because the placement coordinate system is based off blue alliance
            targetCoordinates[0] = self.FieldWidth - targetCoordinates[0]
            allianceFactor = -1
        mandibleMode = self.mandible.state
        if slot < 4: # low-row
            if mandibleMode == "cone":
                offset = self.lowConeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "floor"), cmd.runOnce(lambda: self.mandible.setState('cube'), [self.mandible]), cmd.runOnce(lambda: self.arm.setPosition("optimized"), [self.arm]))
            else:
                offset = self.lowCubeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "floor"), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition("optimized"), [self.arm]))
            robotAngle = self.calculateRobotAngle(self.poseEstimator.getCurrentPose())
            piecePlacement = geometry.Pose2d(geometry.Translation2d(x = targetCoordinates[0], y = targetCoordinates[1]), geometry.Rotation2d())
            robotPose = piecePlacement.transformBy(geometry.Transform2d(translation = geometry.Translation2d(x = (-offset * allianceFactor), y = 0), rotation = robotAngle)) # if errors persist, try making the offset positive
        elif slot > 6: # high-row
            # our only placement option is perpindicular to the grid and right up against it
            # first we have to calculate our required robot position
            if mandibleMode == "cone":
                offset = self.highConeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "highConePrep"), ArmConfirmPlacementCommand(self.arm, "highConePlacement"), cmd.runOnce(lambda: self.mandible.setState('cube'), [self.mandible]), cmd.runOnce(lambda: self.arm.setPosition("optimized"), [self.arm]))
            else:
                offset = self.highCubeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "highCube"), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition("optimized"), [self.arm]))
            robotPose = geometry.Pose2d(targetCoordinates[0] + (offset * allianceFactor), targetCoordinates[1], geometry.Rotation2d(-1, 0))
        else: # mid-row
            if mandibleMode == "cone":
                offset = self.midConeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "midConePrep"), ArmConfirmPlacementCommand(self.arm, "midConePlacement"), cmd.runOnce(lambda: self.mandible.setState('cube'), [self.mandible]), cmd.runOnce(lambda: self.arm.setPosition("optimized"), [self.arm]))
            else:
                offset = self.midCubeOffset
                placementSequence = cmd.sequence(ArmConfirmPlacementCommand(self.arm, "midCube"), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition("optimized"), [self.arm]))
            robotAngle = self.calculateRobotAngle(self.poseEstimator.getCurrentPose())
            piecePlacement = geometry.Pose2d(geometry.Translation2d(x = targetCoordinates[0], y = targetCoordinates[1]), geometry.Rotation2d())
            robotPose = piecePlacement.transformBy(geometry.Transform2d(translation = geometry.Translation2d(x = (-offset * allianceFactor), y = 0), rotation = robotAngle))
        return robotPose, placementSequence
    
    def onlySetArmPosition(self, slot: int) -> None:
        if slot < 4: # low-row
            self.arm.setPosition("floor")
        elif slot > 6: # high-row
            if self.mandible.state == "cone":
                self.arm.setPosition("highConePrep")
            else:
                self.arm.setPosition("highCube")
        else: # mid-row
            if self.mandible.state == "cone":
                self.arm.setPosition("midConePrep")
            else:
                self.arm.setPosition("midCube")
        
    def calculateRobotAngle(self, currentPose: geometry.Pose2d) -> geometry.Rotation2d:
        targetCoordinates = self.GridLayout[self.targetGridSlot[0]][self.targetGridSlot[1]]
        return geometry.Rotation2d(targetCoordinates[0] - currentPose.X(), targetCoordinates[1] - currentPose.X())
    
    def end(self, interrupted: bool) -> None:
        self.swerveAutoBuilder.useAllianceColor = True
        return super().end(interrupted)