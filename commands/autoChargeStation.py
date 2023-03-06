import commands2
import wpilib
from wpimath import geometry
import math

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class AutonomousChargeStation(commands2.CommandBase):
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.navx = self.driveTrain.navx
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        return super().execute()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()
    
    def getAngleAboutYAxis(self):
        roll = geometry.Rotation2d(math.radians(self.navx.getRoll())) # robot tilting forward/backward
        pitch = geometry.Rotation2d(math.radians(self.navx.getPitch())) # robot tilting laterally
        yaw = self.poseEstimator.getCurrentPose().rotation()
        rotationContainer = geometry.Rotation3d(roll = roll, pitch = pitch, yaw = yaw)
        rotationDifference = geometry.Pose2d(geometry.Translation2d(), yaw).relativeTo(geometry.Pose2d()).rotation()
        resultantRoll = rotationContainer.rotateBy(geometry.Rotation3d(geometry.Rotation2d(), geometry.Rotation2d(), rotationDifference))
        return resultantRoll.X()