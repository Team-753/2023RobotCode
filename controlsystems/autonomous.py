import wpilib
from wpimath import controller
from wpimath import geometry, kinematics
from pathplannerlib import PathPlanner, PathPlannerTrajectory, PathConstraints, PathPoint
from math import radians
from controlsystems.ppHolonomicDriveController import PPHolonomicDriveController
from controlsystems.commandController import CommandController

''' For intellisense: '''
from subsystems.driveTrain import DriveTrain
from buster2.poseEstimator import PoseEstimatorSubsystem

class Autonomous:
    currentPose = geometry.Pose2d(0, 0, 0)
    autonomousQueue = []
    useAllianceColor = True
    def __init__(self, config: dict, tags: dict, driveTrain: DriveTrain, poseEstimator: PoseEstimatorSubsystem) -> None:
        self.config = config
        self.tags = tags
        self.driveTrain = driveTrain
        self.poseEstimator = poseEstimator
        
        self.maxVel = self.config["autonomousSettings"]["autoVelLimit"]
        self.maxAcc = self.config["autonomousSettings"]["autoAccLimit"]
        self.autoConstraints = PathConstraints(self.maxVel, self.maxAcc)
        '''
        "xPoseToleranceMeters": 0.01,
        "yPoseToleranceMeters": 0.01,
        "thetaPoseToleranceDegrees": 0.25
        '''
        
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
            PathConstraints(4, 3),
            [
                PathPoint(self.currentPose.translation(), geometry.Rotation2d(), self.currentPose.rotation()),
                PathPoint(tagPose.toPose2d().transformBy(geometry.Transform2d(geometry.Translation2d(distance=1, angle=0), geometry.Rotation2d())).translation(), geometry.Rotation2d(), tagPose.rotation().toRotation2d().rotateBy(geometry.Rotation2d(180)))
            ]
        )
        markers = trajectory.getMarkers()
        return ""
    
    def loadFullAuto(self, pathname: str):
        ''''''
        #path = PathPlanner.loadPath(pathname, self.maxVel, self.maxAcc, False)
        
    def startNewFullAuto(self, pathName: str):
        untransformedTrajectoryList = PathPlanner.loadPathGroup(pathName, self.autoConstraints, False)
        transformedTrajectoryList = []
        for trajectory in untransformedTrajectoryList:
            transformedTrajectoryList.append(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, wpilib.DriverStation.getAlliance()))
        transformedTrajectoryList = [PathPlannerTrajectory] # intellisense DELETE LATER
        firstPathPose = transformedTrajectoryList[0].getInitialHolonomicPose()
        
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
    
    def fullAutoWithCommands(self, pathGroup: list[PathPlannerTrajectory]):
        pass

'''
Conceptual Skelton for Full Autnomous path following with commands

- First check using the timer if we are already moving on the path
- if timer is zero, check if we are within the starting constraints for the set path.
- if not in starting contraints, generate a on the fly path to move to the required starting pose.
- 
'''