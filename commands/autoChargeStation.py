import commands2
import wpilib
from wpimath import geometry
import math
import pathplannerlib

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from auto.swerveAutoBuilder import SwerveAutoBuilder

class AutonomousChargeStation(commands2.CommandBase):
    chargeStationCenterX = 3.8862 # in meters
    chargeStationCenterY = 2.743581 # in meters
    FieldWidth = 16.54175
    
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, SwerveAutoBuilder: SwerveAutoBuilder, Constraints: pathplannerlib.PathConstraints) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.constraints = Constraints
        
    def initialize(self) -> None:
        self.swerveAutoBuilder.useAllianceColor = False
    
    def execute(self) -> None:
        return super().execute()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()