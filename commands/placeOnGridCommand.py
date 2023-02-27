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
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, DriveTrain: DriveTrainSubSystem, Constraints: pathplannerlib.PathConstraints, EventMap: dict, DriverJoystick: wpilib.Joystick, isAutonomous: bool, targetGridSlot: List[int, int], config: dict) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm, DriveTrain])
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.eventMap = EventMap
        self.driverJoystick = DriverJoystick
        self.constraints = Constraints
        self.driveTrain = DriveTrain
        self.isAutonomous = isAutonomous
        self.targetGridSlot = targetGridSlot
        self.config = config
        
        
    def initialize(self) -> None:
        self.currentPose = self.poseEstimator.getCurrentPose()
        self.sequence = self.getTargetGridValues(self.targetGridSlot[0], self.targetGridSlot[1])
        self.arm.setPosition(self.sequence[1])
        if self.isAutonomous:
            onLeFlyTJ = pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.currentPose, self.poseEstimator.chassisSpeeds), self.sequence[0]])
            cmd.sequence([self.swerveAutoBuilder.followPath(onLeFlyTJ), self.sequence[2]])
            
            
    
    def execute(self) -> None:
        self.currentPose = self.poseEstimator.getCurrentPose()
        if not self.isAutonomous: # well shit it ain't autonomous that's for sure
            if self.driverJoystick.getRawButton(1): # is the driver pulling the trigger (confirming game piece placement)
                pass
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
                self.driveTrain.joystickDrive(adjustedInputs, self.currentPose)
            
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()
    
    def adjustToRedAlliance(self, xCoordinate: float):
        return self.FieldWidth - xCoordinate
    
    def getTargetGridValues(self, grid: int, slot: int) -> Tuple[geometry.Pose2d, str, List[commands2.Command]]: # returns geometry.Pose2d, armPosition: str, List[commands2.Command]
        targetCoordinates = self.GridLayout[grid][slot]
        if slot > 3: # mid-row
            pass
        elif slot > 6: # high-row
            pass
        else: # low-row
            pass
        
    def calculateRobotAngle(self, currentPose: geometry.Pose2d) -> geometry.Rotation2d:
        targetCoordinates = self.GridLayout[self.targetGridSlot[0]][self.targetGridSlot[1]]
        return geometry.Rotation2d(targetCoordinates[0] - currentPose.x, targetCoordinates[1] - currentPose.y)