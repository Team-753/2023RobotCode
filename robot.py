import wpilib
import json
import os
#from networktables import NetworkTables
import threading
import photonvision
from controlsystems.poseEstimator import PoseEstimatorSubsystem
from controlsystems.operator import Operator
from subsystems.driveTrain import DriveTrain
from controlsystems.autonomous import Autonomous
from wpimath import geometry
from wpilib import shuffleboard
import photonvision

'''cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True
		cond.notify()
NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)'''

class MyRobot(wpilib.TimedRobot):
    aprilTagPreviouslyDetected = False 
    def robotInit(self):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        tagPath = os.path.join(folderPath, '../apriltags.json')
        with open (tagPath, "r") as f1:
            self.tags = json.load(f1)
        self.cameraOne = photonvision.PhotonCamera("cameraOne")
        self.driveTrain = DriveTrain(self.config)
        self.driverData = shuffleboard.Shuffleboard.getTab("Driver")
        self.operator = Operator(self.config)
        self.autonomous = Autonomous(self.config, self.tags, self.driveTrain, PoseEstimatorSubsystem(self.cameraOne, self.driveTrain, self.tags, geometry.Pose2d()))
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        self.autonomous.followAprilTag()
    
    def autonomousInit(self):
        pass
        
    def autonomousPeriodic(self):
        self.autonomous.bigDaddy()
        
    def teleopInit(self):
        pass
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        self.autonomous.bigDaddy()
    
    def disabledPeriodic(self):
        ''' Runs while the robot is idle '''
        self.operator.checkInputDevices()
        # checking whether or not an apriltag is detected
        '''result = self.cameraOne.getLatestResult() # getting the latest result from the camera
        if (result.hasTargets() and not self.aprilTagPreviouslyDetected): # checking if the photoncamera actually has any apriltags in view
            id = result.getBestTarget().getFiducialId() # getting the id of the closest apriltag
            self.driverData.add("Apriltag Detected", f"{id}") # adding the detection to driverstation
            self.aprilTagPreviouslyDetected = True
        elif (self.aprilTagPreviouslyDetected): # no apriltag detected :(
            self.driverData.add("Apriltag Detected", "NONE") # adding "NONE" to the driverstation
            self.aprilTagPreviouslyDetected = False'''
    
    def disabledInit(self) -> None:
        self.driveTrain.coast()
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)