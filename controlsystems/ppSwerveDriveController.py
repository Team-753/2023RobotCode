from controlsystems.ppHolonomicDriveController import PPHolonomicDriveController
from wpimath import controller, geometry
from math import radians

class PPSwerveDriveController:
    
    def __init__(self, config: dict) -> None:
        self.config = config
        xController = controller.PIDController()
        yController = controller.PIDController()
        thetaController = controller.ProfiledPIDControllerRadians()
        tolerance = geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"])))
        self.autoController = PPHolonomicDriveController(xController, yController, thetaController, tolerance)