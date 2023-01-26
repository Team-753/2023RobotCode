from controlsystems.poseEstimator import PoseEstimatorSubsystem


class AutoSwerveDrive:
    
    def __init__(self, config: dict, poseEstimator: PoseEstimatorSubsystem) -> None:
        self.config = config
        self.poseEstimator = poseEstimator
        self.translationConstants = 