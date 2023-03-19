import wpilib  
from wpimath import estimator, geometry
import photonvision
from subsystems.driveTrain import DriveTrainSubSystem
import math
import robotpy_apriltag
import commands2
from wpilib import SmartDashboard, shuffleboard
from typing import List

class PoseEstimatorSubsystem(commands2.SubsystemBase):
    ''' The infrastructure for estimating robot pose based off of vision and wheel odometry data '''
    stateStdDevs = 0.05, 0.05, math.radians(5)
    visionMeasurementStdDevs = 0.5, 0.5, math.radians(30)
    field = wpilib.Field2d()
    previousPipelineResultTimeStamp = 0 # useless for now...
    useAprilTagThresholdMeters = 1.5
    deltaTilt = 0
    apriltagFieldLayout = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2023ChargedUp)
    apriltagFieldLayout.setOrigin(robotpy_apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide)
    alliance = wpilib.DriverStation.Alliance.kBlue
    def __init__(self, photonCamera: photonvision.PhotonCamera, driveTrain: DriveTrainSubSystem, initialPose: geometry.Pose2d, config: dict) -> None:
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
        self.photonCamera = photonCamera
        cameraParams = self.config["RobotDimensions"]["PhotonCameras"][self.photonCamera.getCameraName()]
        self.cameraTransformation = geometry.Transform3d(geometry.Translation3d(cameraParams["x"], cameraParams["y"], cameraParams["z"]), geometry.Rotation3d(math.radians(cameraParams["roll"]), math.radians(cameraParams["pitch"]), math.radians(cameraParams["yaw"])))
        
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.driveTrain.KINEMATICS, 
                                                                 self.driveTrain.getNAVXRotation2d(), 
                                                                 self.driveTrain.getSwerveModulePositions(), 
                                                                 initialPose, 
                                                                 self.stateStdDevs,
                                                                 self.visionMeasurementStdDevs)
        #self.tab = shuffleboard.Shuffleboard.getTab("Field")
        wpilib.SmartDashboard.putData("Field", self.field)
        #self.tab.add("Field", self.field).withPosition(5, 0).withSize(6, 4)
        self.tilt = 0
        self.tiltTimeStamp = 0
        self.YTimer = wpilib.Timer()
        self.YTimer.start()
        
    def periodic(self) -> None:
        ''' Call this function with every iteration of your autonomous and teleop loop. '''
        
        '''
        For this next section of code we are referencing both of our cameras to find the best apriltag to esimate our position off of, if one is available
        '''
        # NOTE: This algorithm is way too expensive. Anyway, TODO: Implement dual-camera pose throwaway/averaging
        currentPose = self.getCurrentPose()
        pipelineResult = self.photonCamera.getLatestResult()
        resultTimeStamp = pipelineResult.getTimestamp()
        if (resultTimeStamp > self.previousPipelineResultTimeStamp and pipelineResult.hasTargets()):
            target = pipelineResult.getBestTarget()
            fiducialId = target.getFiducialId()
            if target.getPoseAmbiguity() <= 0.15 and fiducialId > 0 and fiducialId < 9:
                camToTarget = target.getBestCameraToTarget()
                norm = camToTarget.translation().toTranslation2d().norm()
                wpilib.SmartDashboard.putNumber("distance to tag", camToTarget.translation().toTranslation2d().norm())
                if norm < self.useAprilTagThresholdMeters or self.isDisabled:
                    targetPose = self.getTagPose(fiducialId) # need 3d poses of each apriltag id
                    camPose = targetPose.transformBy(camToTarget.inverse())
                    robotPose = camPose.transformBy(self.cameraTransformation)
                    robotPose2d = robotPose.toPose2d()
                    self.previousPipelineResultTimeStamp = resultTimeStamp 
                    self.poseEstimator.addVisionMeasurement(robotPose2d, resultTimeStamp)
                    
        self.poseEstimator.update(
            self.driveTrain.getNAVXRotation2d(),
            self.driveTrain.getSwerveModulePositions())
        self.field.setRobotPose(currentPose)
        SmartDashboard.putNumber("X Position", currentPose.X())
        SmartDashboard.putNumber("Y Position", currentPose.Y())
        SmartDashboard.putNumber("Rotation", currentPose.rotation().degrees())
        oldTilt = self.tilt
        self.tilt = self.getTilt()
        oldTimeStamp = self.tiltTimeStamp
        self.tiltTimeStamp = self.YTimer.get()
        self.deltaTilt = (self.tilt - oldTilt) / (self.tiltTimeStamp - oldTimeStamp) # in degrees per second
        SmartDashboard.putNumber("Delta Tilt", self.deltaTilt)
        wpilib.SmartDashboard.putNumber("Field Relative Tilt", self.tilt)
    
    def getTilt(self):
        pitch = math.radians(self.navx.getPitch()) # robot tilting forward/backward
        roll = math.radians(self.navx.getRoll()) # robot tilting side to side
        yaw = self.getCurrentPose().rotation().radians() # yaw is independent of pitch and roll axes, navx automatically accounts for that, we can use this value to rotate to charge station reference
        tilt = roll * math.cos(yaw) + pitch * math.sin(yaw)
        return math.degrees(tilt)
        
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
        return self.apriltagFieldLayout.getTagPose(id)
    
    def setAlliance(self, alliance: wpilib.DriverStation.Alliance):
        self.alliance = alliance
        if alliance == wpilib.DriverStation.Alliance.kBlue:
            self.apriltagFieldLayout.setOrigin(robotpy_apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide)
        else:
            self.apriltagFieldLayout.setOrigin(robotpy_apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide)