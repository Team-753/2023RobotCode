from wpimath import controller, geometry, kinematics
import math
from typing import List
import commands2
from pathplannerlib import PathPlannerTrajectory, controllers
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from wpilib import DriverStation, Timer
from wpimath import controller

class PPSwerveDriveController(commands2.CommandBase):
    
    def __init__(self, trajectory: PathPlannerTrajectory, driveTrain: DriveTrainSubSystem, poseEstimator: PoseEstimatorSubsystem, xController: controller.PIDController, yController: controller.PIDController, thetaController: controller.PIDController, useAllianceColor: bool, tolerance: geometry.Pose2d) -> None:
        super().__init__()
        self.trajectory = trajectory
        self.useAllianceColor = useAllianceColor
        self.poseEstimator = poseEstimator
        self.controller = controllers.PPHolonomicDriveController(xController, yController, thetaController)
        self.controller.setTolerance(tolerance)
        self.driveTrain = driveTrain
        self.addRequirements(driveTrain)
        
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
        print("initializing swerve controller command")
    
    def execute(self):
        print("running auto")
        currentTime = self.timer.get()
        desiredState = self.transformedTrajectory.sample(currentTime)
        currentPose = self.poseEstimator.getCurrentPose()
        
        targetChassisSpeeds = self.controller.calculate(currentPose, desiredState)
        self.driveTrain.autoDrive(targetChassisSpeeds, currentPose)
        
    def end(self, interruped: bool):
        print("auto done")
        self.timer.stop()
        
        if (interruped or abs(self.transformedTrajectory.getEndState().velocity < 0.1)):
            self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, 0, 0), geometry.Pose2d())
            
    def isFinished(self):
        return self.timer.hasElapsed(self.transformedTrajectory.getTotalTime())