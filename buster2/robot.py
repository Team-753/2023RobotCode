import wpilib
import json
import os
import math
#from networktables import NetworkTables
import threading
import photonvision
from driveTrain import DriveTrain
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
        self.joystick = wpilib.Joystick(0)
        
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
        self.driveTrain.navx.reset()
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        x, y, z = self.evaluateDeadzones([self.joystick.getX(), self.joystick.getY(), self.joystick.getZ()])
        self.driveTrain.reportSwerves()
        if (x == 0 and y == 0 and z == 0):
            self.driveTrain.coast()
        else:  
            self.driveTrain.joystickDrive(x, y, z, True)
            print(f"x: {x}, y: {y}, z: {z}")
    
    def evaluateDeadzones(self, inputs):
        '''This method takes in a list consisting of x input, y input, z input, arm input, and winch input.
        The magnitude of the units has to be less than 1.
        Returns the list of inputs with zero in place of values less than their respective deadzones.'''
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = self.config["driverStation"]["joystickDeadZones"][(list(self.config["driverStation"]["joystickDeadZones"])[idx])]
            if abs(input) > threshold: 
                adjustedValue = (abs(input) - threshold) / (1 - threshold)
                if input < 0 and adjustedValue != 0:
                    adjustedValue = -adjustedValue
            else:
                adjustedValue = 0
            adjustedInputs.append(adjustedValue)
        return adjustedInputs
    
    def disabledPeriodic(self):
        ''' Runs while the robot is idle '''
        self.driveTrain.reportSwerves()
    
    def disabledInit(self) -> None:
        '''self.driveTrain.coast()'''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)