from wpimath import geometry, kinematics, controller
from pathplannerlib import PathPlannerTrajectory
from math import pi

class PPHolonomicDriveController:
    ''' A holonomic drive controller adapted to pathplanner. Calculates the necessary chassis speeds to reach the desired path point.'''
    translationError = geometry.Translation2d()
    rotationError = geometry.Rotation2d()
    enabled = True # default
    
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
    
    def setEnabled(self, enable: bool):
        self.enabled = enable
        
    def setTolerance(self, tolerance: geometry.Pose2d):
        self.tolerance = tolerance
    
    def Calculate(self, currentPose: geometry.Pose2d, referenceState: PathPlannerTrajectory.PathPlannerState):
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