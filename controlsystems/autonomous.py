import wpilib
from wpimath import controller
from wpimath import geometry, kinematics
from pathplannerlib import PathPlanner, PathPlannerTrajectory, PathConstraints, PathPoint
from math import pi

''' For intellisense: '''
import subsystems.driveTrain
import controlsystems.poseEstimator

class Autonomous:
    currentPose = geometry.Pose2d(0, 0, 0)
    def __init__(self, config: dict, tags: dict, driveTrain: subsystems.driveTrain, poseEstimator: controlsystems.poseEstimator, mode: str) -> None:
        self.config = config
        self.alliance = wpilib.DriverStation.getAlliance()
        self.mode = mode
        self.tags = tags
        
        self.maxVel = self.config["autonomousSettings"]["autoVelLimit"]
        self.maxAcc = self.config["autonomousSettings"]["autoAccLimit"]
        
        xController = controller.PIDController()
        yController = controller.PIDController()
        thetaController = controller.ProfiledPIDControllerRadians()
        
        self.autoController = controller.HolonomicDriveController(xController, yController, thetaController)
        '''
        options for modes:
        goto
        autopath
        '''
        
    def goTo(self, x, y, rotation):
        pass
    
    def followAprilTag(self, id: int):
        ''' Stays one meter in front of the corresponding apriltag '''
        tagPose = self.getTargetPose(id)
        trajectory = PathPlanner.generatePath( # NOTE: Mess with heading value later, maybe make it equal the the vector heading?
            PathConstraints(6, 3),
            [
                PathPoint(self.currentPose.translation(), geometry.Rotation2d(), self.currentPose.rotation()),
                PathPoint(tagPose.toPose2d().transformBy(geometry.Transform2d(geometry.Translation2d(distance=1, angle=0), geometry.Rotation2d())).translation(), geometry.Rotation2d(), tagPose.rotation().toRotation2d().rotateBy(geometry.Rotation2d(180)))
            ]
        )
        state = trajectory.sample()
        return ""
    
    def loadPath(self, pathname: str):
        ''''''
        #path = PathPlanner.loadPath(pathname, self.maxVel, self.maxAcc, False)
        
    def getTargetPose(self, id: int):
        tag = self.tags[f"{id}"]
        tagRotation = geometry.Rotation3d(0, 0, tag["theta"])
        tagPose = geometry.Pose3d(tag["x"], tag["y"], tag["z"], tagRotation)
        return tagPose
    
    def bigDaddy(self):
        ''' Does literally everything to do with anything, runs periodically in ALL modes.
        parameters: TBD '''
    
class PPHolonomicDriveController:
    translationError = geometry.Translation2d()
    rotationError = geometry.Rotation2d()
    enabled = True
    
    def __init__(self, xController: controller.PIDController, yController: controller.PIDController, thetaController: controller.PIDController, tolerance: geometry.Pose2d) -> None:
        self.xController = xController
        self.yController = yController
        self.thetaController = thetaController
        thetaController.enableContinuousInput(-pi, pi)
        self.tolerance = tolerance
        
    def atReference(self):
        translationTolerance = self.tolerance.translation()
        rotationTolerance = self.tolerance.rotation()
        return abs(self.translationError.X()) < translationTolerance.X() and abs(self.translationError.Y()) < translationTolerance.Y() and abs(self.rotationError.radians()) < rotationTolerance.radians()
    
    def enable(self, enable: bool):
        self.enabled = enable
    
    def ChassisSpeeds(self, currentPose: geometry.Pose2d, referenceState: PathPlannerTrajectory.PathPlannerState):
        xFF = referenceState.velocity * referenceState.pose.rotation().cos()
        yFF = referenceState.velocity * referenceState.pose.rotation().sin()
        rotationFF = referenceState.holonomicAngularVelocity
        
        self.translationError = referenceState.pose.relativeTo(currentPose).rotation()
        self.rotationError = referenceState.holonomicRotation.rotateBy(currentPose.rotation()) # docs list ".minus" here, not present in robotpy
        
        if (not self.enabled):
            return (kinematics.ChassisSpeeds.fromFieldRelativeSpeeds((xFF, yFF, rotationFF, currentPose.rotation())))
        
        xFeedback = self.xController.calculate(currentPose.X(), referenceState.pose.X())
        yFeedback = self.yController.calculate(currentPose.Y(), referenceState.pose.Y())
        rotationFeedback = self.thetaController.calculate(currentPose.rotation().radians(), referenceState.holonomicRotation.radians())
        return kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.rotation())