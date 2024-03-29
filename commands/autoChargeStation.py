import commands2
from wpimath import geometry, controller, kinematics
import math

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class AutonomousChargeStation(commands2.CommandBase):
    thresholdDegPerSec = -17.5
    tolerance = 1 # +/- one degree
    staticFrictionFFTurn = 0.2
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, secondStage: bool, inverted = False) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.addRequirements(self.driveTrain)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.inverted = inverted
        self.speedMS = 2.25
        if secondStage:
            self.speedMS = 0.375
        
    def initialize(self) -> None:
        self.done = False
        self.angleController.reset()
    
    def execute(self) -> None:
        if self.inverted:
            modifier = -1
        else:
            modifier = 1
        deltaTilt = self.poseEstimator.deltaTilt
        if deltaTilt < self.thresholdDegPerSec:
            self.done = True
        else:
            currentPose = self.poseEstimator.getCurrentPose()
            if self.inverted:
                rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi) # math.pi / 4
            else:
                rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi) # -3 * math.pi / 4
            if rotationFeedback < 0:
                rotFF = -self.staticFrictionFFTurn
            else:
                rotFF = self.staticFrictionFFTurn
            self.driveTrain.autoDrive(kinematics.ChassisSpeeds(modifier * -self.speedMS, 0, rotationFeedback + rotFF), currentPose)
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.xMode()
    
    def isFinished(self) -> bool:
        return self.done