import commands2
import wpilib
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from wpimath import kinematics

class AutoDriveCommand(commands2.CommandBase):
    
    def __init__(self, xVel: float, yVel: float, rotationAngVel: float, timeout: float, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.xVel = xVel
        self.yVel = yVel
        self.rotVel = rotationAngVel
        self.timeout = timeout
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.timer = wpilib.Timer()
        self.addRequirements(self.driveTrain)
        
    def initialize(self) -> None:
        self.timer.restart()
    
    def execute(self) -> None:
        self.driveTrain.autoDrive(kinematics.ChassisSpeeds(self.xVel, self.yVel, self.rotVel), self.poseEstimator.getCurrentPose())
    
    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.timeout)
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()
        self.timer.stop()