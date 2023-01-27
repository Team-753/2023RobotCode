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
    def robotInit(self):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        self.driveTrain = DriveTrain(self.config)
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        '''self.autonomous.followAprilTag()'''
    
    def autonomousInit(self):
        pass
        
    def autonomousPeriodic(self):
        # self.autonomous.bigDaddy()
        pass
        
    def teleopInit(self):
        pass
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        # self.autonomous.bigDaddy()
    
    def disabledPeriodic(self):
        ''' Runs while the robot is idle '''
        
    
    def disabledInit(self) -> None:
        '''self.driveTrain.coast()'''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)