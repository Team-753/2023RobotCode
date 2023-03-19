import commands2
from wpimath import controller, kinematics, geometry
import math

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class TurnToCommand(commands2.CommandBase):
    tolerance = 1 # +/- one degree
    staticFrictionFFTurn = 0.2
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem, TargetAngle: geometry.Rotation2d) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.addRequirements(self.driveTrain)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.targetAngle = TargetAngle
        
    def initialize(self) -> None:
        self.done = False
        self.angleController.reset()
    
    def execute(self) -> None:
        if self.angleController.atSetpoint():
            self.done = True
        else:
            currentPose = self.poseEstimator.getCurrentPose()
            rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), self.targetAngle.radians())
            if rotationFeedback < 0:
                rotFF = -self.staticFrictionFFTurn
            else:
                rotFF = self.staticFrictionFFTurn
            self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, 0, rotationFeedback + rotFF), currentPose)
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()
    
    def isFinished(self) -> bool:
        return self.done