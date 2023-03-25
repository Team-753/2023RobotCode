import commands2
import wpilib
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from wpimath import kinematics

class DriveUntilGroundLevelCommand(commands2.CommandBase):
    
    def __init__(self, xVel: float, reversed: bool, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.xVel = xVel
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.timer = wpilib.Timer()
        self.inversionFactor = 1
        if reversed:
            self.inversionFactor = -1
        self.addRequirements(self.driveTrain)
    
    def execute(self) -> None:
        self.driveTrain.autoDrive(kinematics.ChassisSpeeds(-self.xVel * self.inversionFactor, 0, 0), self.poseEstimator.getCurrentPose())
        if self.poseEstimator.tilt < 2: # angle is already abs()
            self.timer.start()
        else:
            self.timer.reset()
    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.timeout)
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()
        self.timer.stop()