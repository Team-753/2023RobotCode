from controlsystems.poseEstimator import PoseEstimatorSubsystem


class AutoSwerveDrive:
    
    def __init__(self, config: dict, poseEstimator: PoseEstimatorSubsystem) -> None:
        self.config = config
        self.poseEstimator = poseEstimator
        self.translationConstants = self.config["autonomousSettings"]["translationPIDConstants"]
        self.rotationConstants = self.config["autonomousSettings"]["rotationPIDConstants"]