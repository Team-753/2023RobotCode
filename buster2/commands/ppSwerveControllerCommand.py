from controllers.ppHolonomicDriveController import PPHolonomicDriveController
from wpimath import controller, geometry, kinematics
import math
import commands2
from pathplannerlib import PathPlannerTrajectory
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from wpilib import DriverStation, Timer

class PPSwerveDriveController(commands2.SubsystemBase):
    
    def __init__(self, trajectory: PathPlannerTrajectory, driveTrain: DriveTrainSubSystem, poseEstimator: PoseEstimatorSubsystem, xController: controller.PIDController, yController: controller.PIDController, thetaController: controller.PIDController, moduleStateOutput, useAllianceColor: bool, tolerance: geometry.Pose2d) -> None:
        super.__init__()
        if (useAllianceColor):
            self.transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance())
        else:
            self.transformedTrajectory = trajectory
        self.poseEstimator = poseEstimator
        self.controller = PPHolonomicDriveController(xController, yController, thetaController, tolerance)
        self.driveTrain = driveTrain
        
        self.Timer = Timer()
        self.Timer.start()
        '''
        self.config = config
        xController = controller.PIDController()
        yController = controller.PIDController()
        thetaController = controller.ProfiledPIDControllerRadians()
        tolerance = geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"])))
        self.autoController = PPHolonomicDriveController(xController, yController, thetaController, tolerance)
        '''
    
    def execute(self):
        currentTime = self.Timer.get()
        desiredState = self.transformedTrajectory.sample(currentTime)
        currentPose = self.poseEstimator.getCurrentPose()
        
        targetChassisSpeeds = self.controller.Calculate(currentPose, desiredState)
        self.driveTrain.drive(targetChassisSpeeds)
        
    def end(self, interruped: bool):
        self.Timer.stop()
        
        if (interruped or abs(self.transformedTrajectory.getEndState().velocity < 0.1)):
            self.driveTrain.drive(kinematics.ChassisSpeeds(0, 0, 0))
            
    def isFinished(self):
        return self.Timer.hasElapsed(self.transformedTrajectory.getTotalTime())