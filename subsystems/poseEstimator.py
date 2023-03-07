import wpilib  
from wpimath import estimator, geometry
import photonvision
from subsystems.driveTrain import DriveTrainSubSystem
import math
import commands2
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
    useVision = True
    isDisabled = True
    craziestVisionPose = geometry.Pose2d()
    
    def __init__(self, photonCameras: List[photonvision.PhotonCamera], driveTrain: DriveTrainSubSystem, initialPose: geometry.Pose2d, config: dict) -> None:
        ''' Initiates the PoseEstimator Subsystem
        
            :param photonCamera: The chosen PhotonCamera object.
            :param driveTrain: The created drivetrain class.
            :param initialPose: Where on the field and in what orientation the robot starts in the form of a Pose2d object.
            :param config: The serialized robot configuration file. '''
        super().__init__()

        self.driveTrain = driveTrain
        self.navx = self.driveTrain.navx
        self.config = config
        self.tags = config["Apriltags"]
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
        #self.tab = shuffleboard.Shuffleboard.getTab("Field")
        wpilib.SmartDashboard.putData("Field", self.field)
        #self.tab.add("Field", self.field).withPosition(5, 0).withSize(6, 4)
        self.YRotation = geometry.Rotation2d()
        self.YTimeStamp = 0
        self.YTimer = wpilib.Timer()
        self.YTimer.start()
        
    def periodic(self) -> None:
        ''' Call this function with every iteration of your autonomous and teleop loop. '''
        
        '''
        For this next section of code we are referencing both of our cameras to find the best apriltag to esimate our position off of, if one is available
        '''
        # NOTE: This algorithm is way too expensive. Anyway, TODO: Implement dual-camera pose throwaway/averaging
        # NOTE: GIANT BREAKTHROUGH MENTALLY: AFTER NAVX HAS BEEN ZEROED BY VISION, ONLY RELY ON NAVX ROTATION AND USE THAT TO FILTER TAG POSES
        cameraIndex = 0
        for camera in self.photonCameras:
            pipelineResult = camera.getLatestResult()
            resultTimeStamp = pipelineResult.getTimestamp()
            if (resultTimeStamp > self.previousPipelineResultTimeStamp and pipelineResult.hasTargets()):
                target = pipelineResult.getBestTarget()
                fiducialId = target.getFiducialId()
                if target.getPoseAmbiguity() <= 0.10 and fiducialId > 0 and fiducialId < 9:
                    targetPose = self.getTagPose(fiducialId) # need 3d poses of each apriltag id
                    camToTarget = target.getBestCameraToTarget()
                    camPose = targetPose.transformBy(camToTarget.inverse())
                    robotPose = camPose.transformBy(self.cameraTransformations[cameraIndex])
                    robotPose2d = robotPose.toPose2d()
                    self.previousPipelineResultTimeStamp = resultTimeStamp
                    if self.isDisabled: # are we disabled
                        self.poseEstimator.addVisionMeasurement(robotPose2d, resultTimeStamp)
                    elif abs(self.getCurrentPose().relativeTo(robotPose2d).rotation().degrees()) < 2: # is the vision pose estimate within 2 degrees of our navx estimate
                        self.poseEstimator.addVisionMeasurement(robotPose2d, resultTimeStamp)
                            
        swerveModuleStates = self.driveTrain.getSwerveModulePositions()
        
        self.poseEstimator.update(
            self.driveTrain.getNAVXRotation2d(),
            swerveModuleStates)
        currentPose = self.getCurrentPose()
        self.field.setRobotPose(currentPose)
        SmartDashboard.putNumber("X Position", currentPose.X())
        SmartDashboard.putNumber("Y Position", currentPose.Y())
        SmartDashboard.putNumber("Rotation", currentPose.rotation().degrees())
        
        oldYRotation = self.YRotation
        self.YRotation = self.getAngleAboutYAxis()
        oldTimeStamp = self.YTimeStamp
        self.YTimeStamp = self.YTimer.get()
        deltaYTheta = (self.YRotation.degrees() - oldYRotation.degrees()) / (self.YTimeStamp - oldTimeStamp) # in degrees per second
        SmartDashboard.putNumber("Field Orient Y Theta", self.YRotation.degrees())
        SmartDashboard.putNumber("Delta Y Theta", deltaYTheta)
        
        
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
    
    def getAngleAboutYAxis(self):
        roll = geometry.Rotation2d(math.radians(self.navx.getRoll())) # robot tilting forward/backward
        pitch = geometry.Rotation2d(math.radians(self.navx.getPitch())) # robot tilting laterally
        yaw = self.getCurrentPose().rotation()
        rotationContainer = geometry.Rotation3d(roll, pitch, yaw)
        rotationDifference = geometry.Pose2d(geometry.Translation2d(), yaw).relativeTo(geometry.Pose2d()).rotation()
        resultantRoll = rotationContainer.rotateBy(geometry.Rotation3d(geometry.Rotation2d(), geometry.Rotation2d(), rotationDifference))
        return geometry.Rotation2d(resultantRoll.X())