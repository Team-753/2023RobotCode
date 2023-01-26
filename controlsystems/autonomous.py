import wpilib
from wpimath import controller
from wpimath import geometry, kinematics
from pathplannerlib import PathPlanner, PathPlannerTrajectory, PathConstraints, PathPoint
from math import radians
from controlsystems.ppHolonomicDriveController import PPHolonomicDriveController
from controlsystems.commandController import CommandController

''' For intellisense: '''
from subsystems.driveTrain import DriveTrain
from controlsystems.poseEstimator import PoseEstimatorSubsystem

class Autonomous:
    currentPose = geometry.Pose2d(0, 0, 0)
    def __init__(self, config: dict, tags: dict, driveTrain: DriveTrain, poseEstimator: PoseEstimatorSubsystem) -> None:
        self.config = config
        self.alliance = wpilib.DriverStation.getAlliance()
        self.tags = tags
        self.driveTrain = driveTrain
        self.poseEstimator = poseEstimator
        
        self.maxVel = self.config["autonomousSettings"]["autoVelLimit"]
        self.maxAcc = self.config["autonomousSettings"]["autoAccLimit"]
        '''
        "xPoseToleranceMeters": 0.01,
        "yPoseToleranceMeters": 0.01,
        "thetaPoseToleranceDegrees": 0.25
        '''
        xController = controller.PIDController()
        yController = controller.PIDController()
        thetaController = controller.ProfiledPIDControllerRadians()
        tolerance = geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"])))
        self.autoController = PPHolonomicDriveController(xController, yController, thetaController, tolerance)
        
        self.commandController = CommandController()
        
        '''
        options for modes:
        goto
        autopath
        '''
    def followAprilTag(self, id: int):
        ''' --TESTING-- Stays one meter in front of the corresponding apriltag '''
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
    
    def loadFullAuto(self, pathname: str):
        ''''''
        #path = PathPlanner.loadPath(pathname, self.maxVel, self.maxAcc, False)
        
    def getTargetPose(self, id: int):
        tag = self.tags[f"{id}"]
        tagRotation = geometry.Rotation3d(0, 0, tag["theta"])
        tagPose = geometry.Pose3d(tag["x"], tag["y"], tag["z"], tagRotation)
        return tagPose
    
    def bigDaddy(self, mode: str, conditionals: dict):
        ''' Does literally everything to do with anything, runs periodically in ALL modes.
        parameters: TBD '''
        self.poseEstimator.periodic() # always ran
        if (mode == "fullAuto"):
            pass
        elif (mode == "commando"):
            pass
        elif (mode == "aprilTagTesting"):
            pass
        else: # passive auto, doesn't do anything, just keeps track of where the robot is still
            pass