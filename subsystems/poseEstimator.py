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
    velocity = 0
    useAprilTagThresholdMeters = 1.5
    deltaTilt = 0    
    
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
        # NOTE: GIANT BREAKTHROUGH MENTALLY: AFTER NAVX HAS BEEN ZEROED BY VISION, ONLY RELY ON NAVX ROTATION AND USE THAT TO FILTER TAG POSES
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
        SmartDashboard.putNumber("Field Orient Y Theta", self.tilt)
        SmartDashboard.putNumber("Delta Y Theta", self.deltaTilt)
        
        
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
    
    '''def getAngleAboutYAxis(self):
        roll = math.radians(self.navx.getRoll()) # robot tilting forward/backward
        pitch = math.radians(self.navx.getPitch()) # robot tilting laterally
        yaw = self.getCurrentPose().rotation().radians()
        rotationContainer = geometry.Rotation3d(roll, pitch, yaw)
        rotationDifference = geometry.Pose2d(geometry.Translation2d(), geometry.Rotation2d(yaw)).relativeTo(geometry.Pose2d()).rotation()
        resultantRoll = rotationContainer.rotateBy(geometry.Rotation3d(0, 0, rotationDifference.radians()))
        return geometry.Rotation2d(resultantRoll.X())'''

    def getTilt(self):
        pitch = math.radians(self.navx.getPitch())
        roll = math.radians(self.navx.getRoll())
        yaw = self.getCurrentPose().rotation().radians()

        # Convert the Euler angles to quaternions
        q_pitch = [math.cos(pitch/2), math.sin(pitch/2), 0, 0]
        q_roll = [math.cos(roll/2), 0, math.sin(roll/2), 0]
        q_yaw = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]

        # Define the charge station's orientation in the robot frame
        station_orientation = [1, 0, 0]  # along the x-axis

        # Rotate the station orientation vector first about the yaw axis,
        # then about the pitch axis, and finally about the roll axis
        station_orientation = self._quaternion_multiply(self._quaternion_conjugate(q_yaw), station_orientation)
        station_orientation = self._quaternion_multiply(self._quaternion_conjugate(q_pitch), station_orientation)
        station_orientation = self._quaternion_multiply(self._quaternion_conjugate(q_roll), station_orientation)

        # Compute the tilt angle of the charge station
        tilt_station = math.atan2(station_orientation[2], station_orientation[0])
        return math.degrees(tilt_station)
    
    def _quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return [w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2]

    def _quaternion_conjugate(q):
        w, x, y, z = q
        return [w, -x, -y, -z]