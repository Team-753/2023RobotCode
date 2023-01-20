import wpilib
import json
import os
#from networktables import NetworkTables
import threading
import photonvision
from controlsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrain
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
    def robotInit(self):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        self.cameraOne = photonvision.PhotonCamera("cameraOne")
        self.driveTrain = DriveTrain(self.config)
        self.poseEstimator = PoseEstimatorSubsystem(self.cameraOne, self.driveTrain, geometry.Pose2d())
        self.driverData = shuffleboard.Shuffleboard.getTab("Driver")
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        self.driveTrain.drive(0, 0.5, 0, True)
    
    def autonomousInit(self):
        pass
        
    def autonomousPeriodic(self):
        #self.poseEstimator.periodic()
        pass
        
    def teleopInit(self):
        pass
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        #self.poseEstimator.periodic()
        pass
        
    def disabledPeriodic(self):
        ''' Runs while the robot is idle '''
        # checking whether or not an apriltag is detected
        result = self.cameraOne.getLatestResult() # getting the latest result from the camera
        if (result.hasTargets()): # checking if the photoncamera actually has any apriltags in view
            id = result.getBestTarget().getFiducialId() # getting the id of the closest apriltag
            self.driverData.add("Apriltag Detected", f"{id}") # adding the detection to driverstation
        else: # no apriltag detected :(
            self.driverData.add("Apriltag Detected", "NONE") # adding "NONE" to the driverstation
    
    def disabledInit(self) -> None:
        ''''''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)
