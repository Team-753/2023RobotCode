import wpilib  
from wpimath import estimator, geometry
from wpilib import shuffleboard
# import robotpy_apriltag # gone unused for now, may stay that way...
#import photonvision
from os import path
from json import load
from subsystems.driveTrain import DriveTrainSubSystem
import math
import commands2
from wpilib import SmartDashboard

class PoseEstimatorSubsystem(commands2.SubsystemBase):
    ''' The infrastructure for estimating robot pose based off of vision and wheel odometry data '''
    stateStdDevs = 0.5, 0.5, math.radians(10) # change this later
    visionMeasurementStdDevs = 0, 0, 0 # change this later
    field = wpilib.Field2d()
    #previousPipelineResultTimeStamp = 0 # useless for now...
    #camRelRobot = geometry.Transform3d(geometry.Translation3d(0, 0, 0), geometry.Rotation3d())
    def __init__(self, MyRobot: commands2.TimedCommandRobot, driveTrain: DriveTrainSubSystem, initialPose: geometry.Pose2d, config: dict) -> None: # photonCamera: photonvision.PhotonCamera,
        ''' Initiates the PoseEstimator Subsystem
        
            :param photonCamera: The chosen PhotonCamera object.
            :param driveTrain: The created drivetrain class.
            :param initialPose: Where on the field and in what orientation the robot starts in the form of a Pose2d object.'''
        #self.photonCamera = photonCamera # we will eventually import the camera, I am leaving this for intellisense
        super().__init__()
        self.driveTrain = driveTrain
        self.config = config
        
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
        
    def periodic(self):
        ''' Call this function with every iteration of your autonomous and teleop loop. '''
        '''pipelineResult = self.photonCamera.getLatestResult()
        resultTimeStamp = pipelineResult.getTimestamp()
        check = False
        if (resultTimeStamp != self.previousPipelineResultTimeStamp and pipelineResult.hasTargets()):
            self.previousPipelineResultTimeStamp = resultTimeStamp
            target = pipelineResult.getBestTarget()
            fiducialId = target.getFiducialId() # https://github.com/STMARobotics/swerve-test/blob/main/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L78
            if (target.getPoseAmbiguity() <= 0.2 and fiducialId >= 0 and fiducialId < 9):
                targetPose = self.getTagPose(fiducialId) # need 3d poses of each apriltag id
                camToTarget = target.getBestCameraToTarget()
                camPose = targetPose.transformBy(camToTarget.inverse())
                robotPose = camPose.transformBy(self.camRelRobot)
                self.poseEstimator.addVisionMeasurement(robotPose.toPose2d(), resultTimeStamp)
                self.driverData.add("Apriltag Detected", f"{fiducialId}")
                check = True
        if check == False:
            self.driverData.add("Apriltag Deteced", "NONE")'''

            
        self.poseEstimator.update(
            self.driveTrain.getNAVXRotation2d(),
            self.driveTrain.getSwerveModulePositions())
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