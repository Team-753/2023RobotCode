import wpilib
from wpimath import estimator, geometry
from wpilib import shuffleboard
import robotpy_apriltag # gone unused for now, may stay that way...
import photonvision

class PoseEstimatorSubsystem:
    ''' The infrastructure for estimating robot pose based off of vision and wheel odometry data '''
    stateStdDevs = 0, 0, 0 # change this later
    visionMeasurementStdDevs = 0, 0, 0 # change this later
    field = wpilib.Field2d()
    previousPipelineResultTimeStamp = 0 # useless for now...
    def __init__(self, photonCamera: photonvision.PhotonCamera, driveTrain: object, initialPose: geometry.Pose2d) -> None:
        ''' Initiates the PoseEstimator Subsystem
        
            :param photonCamera: The chosen PhotonCamera object.
            :param driveTrain: The created drivetrain class.
            :param initialPose: Where on the field and in what orientation the robot starts in the form of a Pose2d object.'''
        self.photonCamera = photonCamera # we will eventually import the camera, I am leaving this for intellisense
        self.driveTrain = driveTrain
        
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.driveTrain.KINEMATICS, 
                                                                 self.driveTrain.getNAVXRotation2d(), 
                                                                 self.driveTrain.getSwerveModulePositions(), 
                                                                 initialPose, 
                                                                 self.stateStdDevs,
                                                                 self.visionMeasurementStdDevs)
        self.tab = shuffleboard.Shuffleboard.getTab("Odometry")
        #self.tab.addString("Pose", self.getFormattedPose()).withPosition(0, 0).withSize(2, 0)
        self.tab.add("Field", self.field).withPosition(5, 0).withSize(6, 4)
        
    def periodic(self):
        ''' Call this function with every iteration of your autonomous and teleop loop. '''
        pipelineResult = self.photonCamera.getLatestResult()
        resultTimeStamp = pipelineResult.getTimestamp()
        if (resultTimeStamp != self.previousPipelineResultTimeStamp and pipelineResult.hasTargets()):
            self.previousPipelineResultTimeStamp = resultTimeStamp
            target = pipelineResult.getBestTarget()
            fiducialId = target.getFiducialId() # https://github.com/STMARobotics/swerve-test/blob/main/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L78
            if (target.getPoseAmbiguity() <= 0.2 and fiducialId >= 0 and fiducialId < 9):
                targetPose = self.getTargetPose(fiducialId) # need 3d poses of each apriltag id
                camToTarget = target.getBestCameraToTarget()
                camPose = targetPose.transformBy(camToTarget.inverse())
                robotPose = self.camToRobot(camPose)
                self.poseEstimator.addVisionMeasurement(robotPose.toPose2d(), resultTimeStamp)
                
                
            
        self.poseEstimator.update(
            self.driveTrain.getNAVXRotation2d(),
            self.driveTrain.getSwerveModulePositions())
        
        self.field.setRobotPose(self.getCurrentPose()) # TODO-> may need to update shuffleboard with this? Further testing needed.
        
    def getFormattedPose(self):
        pose = self.getCurrentPose()
        return f"{pose.X()}, {pose.Y()}, {pose.rotation().degrees()} Degrees"
    
    def getCurrentPose(self):
        return self.poseEstimator.getEstimatedPosition()
    
    def setCurrentPose(self, newPose: geometry.Pose2d):
        ''' Resets the current pose with the given Pose2d object '''
        self.poseEstimator.resetPosition(newPose)
        
    def resetFieldPosition(self):
        self.setCurrentPose(geometry.Pose2d())
        
    def getTargetPose(self, id: int):
        return geometry.Pose3d()
    
    def camToRobot(self, camPose: geometry.Pose3d):
        return geometry.Pose3d()