import wpilib  
from wpimath import estimator, geometry
from wpilib import shuffleboard
# import robotpy_apriltag # gone unused for now, may stay that way...
import photonvision
from os import path
from json import load
from subsystems.driveTrain import DriveTrainSubSystem
import math
import commands2
from wpilib import SmartDashboard
from typing import List

class PoseEstimatorSubsystem(commands2.SubsystemBase):
    ''' The infrastructure for estimating robot pose based off of vision and wheel odometry data '''
    visionMeasurementStdDevs = 0.005, 0.005, math.radians(0.1) # change this later
    stateStdDevs = 0.01, 0.01, math.radians(0.2)  # change this later
    field = wpilib.Field2d()
    previousPipelineResultTimeStamp = 0 # useless for now...
    camOneRelRobot = geometry.Transform3d(geometry.Translation3d(-0.3302, 0, 0), geometry.Rotation3d(0, 0, 0))
    camTwoRelRobot = geometry.Transform3d(geometry.Translation3d(0, 0, 0), geometry.Rotation3d(0, 0, 0))
    velocity = 0
    heading = geometry.Rotation2d()
    
    def __init__(self, photonCameras: List[photonvision.PhotonCamera], driveTrain: DriveTrainSubSystem, initialPose: geometry.Pose2d, config: dict) -> None:
        ''' Initiates the PoseEstimator Subsystem
        
            :param photonCamera: The chosen PhotonCamera object.
            :param driveTrain: The created drivetrain class.
            :param initialPose: Where on the field and in what orientation the robot starts in the form of a Pose2d object.
            :param config: The serialized robot configuration file. '''
        super().__init__()

        self.driveTrain = driveTrain
        self.config = config
        self.photonCameras = photonCameras
        self.cameraTransformations = []
        for camera in photonCameras:
            cameraParams = self.config["RobotDimensions"]["PhotonCameras"][camera.getCameraName()]
            self.cameraTransformations.append(geometry.Transform3d(geometry.Translation3d(cameraParams["x"], cameraParams["y"], cameraParams["z"]), geometry.Rotation3d(cameraParams["roll"], cameraParams["pitch"], cameraParams["yaw"])))
            
        
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.driveTrain.KINEMATICS, 
                                                                 self.driveTrain.getNAVXRotation2d(), 
                                                                 self.driveTrain.getSwerveModulePositions(), 
                                                                 initialPose, 
                                                                 self.stateStdDevs,
                                                                 self.visionMeasurementStdDevs)
        self.tab = shuffleboard.Shuffleboard.getTab("Odometry")
        #self.driverData = shuffleboard.Shuffleboard.getTab("Driver")
        #self.tab.addString("Pose", self.getFormattedPose()).withPosition(0, 0).withSize(2, 0)
        self.tab.add("Field", self.field).withPosition(5, 0).withSize(6, 4)
        #self.driverData.add("Apriltag Detected", "NONE").withPosition(3, 0)
        
    def periodic(self) -> None:
        ''' Call this function with every iteration of your autonomous and teleop loop. '''
        
        '''
        For this next section of code we are referencing both of our cameras to find the best apriltag to esimate our position off of, if one is available
        '''
        '''targetList = [] # creating a list to hold our targets and their given ambiguity
        for camera in self.photonCameras: # indexing through the list
            pipelineResult = camera.getLatestResult()
            resultTimeStamp = pipelineResult.getTimestamp()
            if (resultTimeStamp != self.previousPipelineResultTimeStamp and pipelineResult.hasTargets()):
                target = pipelineResult.getBestTarget()
                targetList.append([pipelineResult, target.getPoseAmbiguity()])
        lowestVal = 1
        for apriltag in targetList:
            if apriltag[1] < lowestVal and apriltag[1] != -1:
                lowestVal = apriltag[1]
        for apriltag in targetList:
            i = 0
            if apriltag[1] == lowestVal: # we have found the "best" apriltag to reference from
                pipelineResult = apriltag[0]
                resultTimeStamp = pipelineResult.getTimestamp()
                self.previousPipelineResultTimeStamp = resultTimeStamp
                target = pipelineResult.getBestTarget()
                fiducialId = target.getFiducialId() # https://github.com/STMARobotics/swerve-test/blob/main/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L78
                if (target.getPoseAmbiguity() <= 0.2 and fiducialId >= 0 and fiducialId < 9):
                    targetPose = self.getTagPose(fiducialId) # need 3d poses of each apriltag id
                    camToTarget = target.getBestCameraToTarget()
                    camPose = targetPose.transformBy(camToTarget.inverse())
                    robotPose = camPose.transformBy(self.cameraTransformations[i])
                    robotPose2d = robotPose.toPose2d()
                    self.poseEstimator.addVisionMeasurement(robotPose2d, resultTimeStamp)
                    difference = robotPose2d.rotation().degrees() + 180 - self.driveTrain.getNAVXRotation2d().degrees() + 180
                    if (abs(difference) > 1): # our navx is off by over a degree as calculated by an apriltag, this is very useful for the start of the match
                        self.driveTrain.navxOffset = difference # setting the new navx offset
            else:
                i += 1'''
        swerveModuleStates = self.driveTrain.getSwerveModulePositions()
        derivedState = self.driveTrain.KINEMATICS.toTwist2d(swerveModuleStates[0], swerveModuleStates[1], swerveModuleStates[2], swerveModuleStates[3])
        self.velocity = math.hypot(derivedState.dx, derivedState.dy)
        self.heading = geometry.Rotation2d(math.atan2(derivedState.dy, derivedState.dx))
        
        self.poseEstimator.update(
            self.driveTrain.getNAVXRotation2d(),
            swerveModuleStates)
        currentPose = self.getCurrentPose()
        SmartDashboard.putNumber("X Position", currentPose.X())
        SmartDashboard.putNumber("Y Position", currentPose.Y())
        SmartDashboard.putNumber("Rotation", currentPose.rotation().degrees())
        
    def getFormattedPose(self):
        pose = self.getCurrentPose()
        return f"{pose.X()}, {pose.Y()}, {pose.rotation().degrees()} Degrees"
    
    def getCurrentPose(self):
        return self.poseEstimator.getEstimatedPosition()
    
    def setCurrentPose(self, newPose: geometry.Pose2d):
        ''' Resets the current pose with the given Pose2d object '''
        self.poseEstimator.resetPosition(self.driveTrain.getNAVXRotation2d(), self.driveTrain.getSwerveModulePositions(), newPose)
        
    def resetFieldPosition(self):
        self.setCurrentPose(geometry.Pose2d())
        
    def getTagPose(self, id: int) -> geometry.Pose3d:
        ''' Returns the 3D pose of the requested apriltag '''
        tag = self.tags[f"{id}"]
        tagRotation = geometry.Rotation3d(0, 0, tag["theta"])
        tagPose = geometry.Pose3d(tag["x"], tag["y"], tag["z"], tagRotation)
        return tagPose