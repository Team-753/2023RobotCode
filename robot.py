import wpilib
import json
import os
from networktables import NetworkTables
import threading
import photonvision
from poseEstimator import PoseEstimatorSubsystem
from driveTrain import DriveTrain
from wpimath import geometry

cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True
		cond.notify()

NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        self.cameraOne = photonvision.PhotonCamera("cameraOne")
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        self.driveTrain = DriveTrain(self.config)
        self.poseEstimator = PoseEstimatorSubsystem(self.cameraOne, self.driveTrain, geometry.Pose2d())
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        return super().testPeriodic()
    
    def autonomousInit(self):
        pass
        
    def autonomousPeriodic(self):
        self.poseEstimator.periodic()
        
    def teleopInit(self):
        pass
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        self.poseEstimator.periodic()
        
    def disabledPeriodic(self):
        ''''''
    
    def disabledInit(self) -> None:
        ''''''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)
