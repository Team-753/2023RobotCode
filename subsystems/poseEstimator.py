import wpilib  
from wpimath import estimator, geometry
import photonvision
from subsystems.driveTrain import DriveTrainSubSystem
import math
import commands2
import robotpy_apriltag
from wpilib import SmartDashboard, shuffleboard
from typing import List
class PoseEstimatorSubsystem(commands2.SubsystemBase):
    ''' The infrastructure for estimating robot pose based off of vision and wheel odometry data '''
    stateStdDevs = 0.05, 0.05, math.radians(5)
    visionMeasurementStdDevs = 0.5, 0.5, math.radians(30)
    field = wpilib.Field2d()
    previousPipelineResultTimeStamp = 0 # useless for now...
    velocity = 0
    heading = geometry.Rotation2d()
    mostRecentVisionPose = geometry.Pose2d()
    
    def __init__(self, photonCameras: List[photonvision.PhotonCamera], driveTrain: DriveTrainSubSystem, initialPose: geometry.Pose2d, config: dict) -> None:
        ''' Initiates the PoseEstimator Subsystem
        
            :param photonCamera: The chosen PhotonCamera object.
            :param driveTrain: The created drivetrain class.
            :param initialPose: Where on the field and in what orientation the robot starts in the form of a Pose2d object.
            :param config: The serialized robot configuration file. '''
        super().__init__()

        self.driveTrain = driveTrain
        self.config = config
        self.tags = config["Apriltags"]
        self.photonCameras = photonCameras
        self.cameraFinalList = []
        for camera in photonCameras:
            cameraParams = self.config["RobotDimensions"]["PhotonCameras"][camera.getCameraName()]
            transformation = geometry.Transform3d(geometry.Translation3d(cameraParams["x"], cameraParams["y"], cameraParams["z"]), geometry.Rotation3d(cameraParams["roll"], cameraParams["pitch"], cameraParams["yaw"]))
            self.cameraFinalList.append((camera, transformation))
        
            
        
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.driveTrain.KINEMATICS, 
                                                                 self.driveTrain.getNAVXRotation2d(), 
                                                                 self.driveTrain.getSwerveModulePositions(), 
                                                                 initialPose, 
                                                                 self.stateStdDevs,
                                                                 self.visionMeasurementStdDevs)
        fieldLayout = robotpy_apriltag.AprilTagField.k2023ChargedUp
        self.photonPoseEstimator = photonvision.RobotPoseEstimator(fieldLayout, photonvision.PoseStrategy.LOWEST_AMBIGUITY, self.cameraFinalList) # update this eventually to multi-tag PNP
        #self.tab = shuffleboard.Shuffleboard.getTab("Field")
        wpilib.SmartDashboard.putData("Field", self.field)
        #self.tab.add("Field", self.field).withPosition(5, 0).withSize(6, 4)
        
    def periodic(self) -> None:
        ''' Call this function with every iteration of your autonomous and teleop loop. '''
        
        #self.photonPoseEstimator.setReferencePose(geometry.Pose3d(self.getCurrentPose()))
        estimate = self.photonPoseEstimator.update()
        if estimate[1] > self.previousPipelineResultTimeStamp:
            estimate[0].toPose2d()
            self.poseEstimator.addVisionMeasurement(estimate[0].toPose2d(), estimate[1])
        swerveModuleStates = self.driveTrain.getSwerveModulePositions()

        
        self.poseEstimator.update(
            self.driveTrain.getNAVXRotation2d(),
            swerveModuleStates)
        currentPose = self.getCurrentPose()
        self.field.setRobotPose(currentPose)
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