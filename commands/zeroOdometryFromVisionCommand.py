import commands2
from subsystems.poseEstimator import PoseEstimatorSubsystem

class ZeroOdometryFromVisionCommand(commands2.CommandBase):
    
    def __init__(self, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.poseEstimator = PoseEstimator
        self.finished = False
        self.setName("Zero From Vision")
    
    def initialize(self) -> None:
        self.poseEstimator.setCurrentPose(self.poseEstimator.mostRecentVisionPose)
        self.finished = True
    
    def isFinished(self) -> bool:
        return self.finished
    
    def runsWhenDisabled(self) -> bool:
        return True