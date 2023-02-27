from controllers.ppHolonomicDriveController import PPHolonomicDriveController
from wpimath import controller, geometry, kinematics
import math
from typing import List
import commands2
from pathplannerlib import PathPlannerTrajectory
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from wpilib import DriverStation, Timer
from wpimath import controller

class PPSwerveDriveController(commands2.CommandBase):
    
    def __init__(self, trajectory: PathPlannerTrajectory, driveTrain: DriveTrainSubSystem, poseEstimator: PoseEstimatorSubsystem, xController: controller.PIDController, yController: controller.PIDController, thetaController: List[dict], useAllianceColor: bool, tolerance: geometry.Pose2d) -> None:
        super().__init__()
        self.trajectory = trajectory
        self.useAllianceColor = useAllianceColor
        self.poseEstimator = poseEstimator
        thetaController = controller.ProfiledPIDControllerRadians(thetaController[0]["kP"], thetaController[0]["kI"], thetaController[0]["kD"], thetaController[1], thetaController[0]["period"])
        self.controller = PPHolonomicDriveController(xController, yController, thetaController, tolerance)
        self.driveTrain = driveTrain
        self.addRequirements(self.driveTrain)
        
        self.timer = Timer()
        '''
        self.config = config
        xController = controller.PIDController()
        yController = controller.PIDController()
        thetaController = controller.ProfiledPIDControllerRadians()
        tolerance = geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"])))
        self.autoController = PPHolonomicDriveController(xController, yController, thetaController, tolerance)
        '''
    
    def initialize(self) -> None:
        if (self.useAllianceColor):
            self.transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(self.trajectory, DriverStation.getAlliance())
        else:
            self.transformedTrajectory = self.trajectory
            
        self.timer.reset()
        self.timer.start()
    
    def execute(self):
        currentTime = self.timer.get()
        desiredState = self.transformedTrajectory.sample(currentTime)
        currentPose = self.poseEstimator.getCurrentPose()
        
        targetChassisSpeeds = self.controller.Calculate(currentPose, desiredState)
        self.driveTrain.drive(targetChassisSpeeds)
        
    def end(self, interruped: bool):
        self.timer.stop()
        
        if (interruped or abs(self.transformedTrajectory.getEndState().velocity < 0.1)):
            self.driveTrain.drive(kinematics.ChassisSpeeds(0, 0, 0))
            
    def isFinished(self):
        return self.timer.hasElapsed(self.transformedTrajectory.getTotalTime())