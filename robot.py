import wpilib
import json
import os
from networktables import NetworkTables
import navx
import threading
import photonvision
from driverStation import driverStation
from driveTrain import driveTrain

cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True
		cond.notify()

NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True) # this is also broken
smartDash = NetworkTables.getTable('SmartDashboard')
#mainCamera = photonvision.PhotonCamera("mainCamera")

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        self.driverStation = driverStation(self.config)
        self.navx = navx.AHRS.create_spi()
        self.driveTrain = driveTrain(self.config, self.navx)
        self.DEBUGSTATEMENTS = False
        
    def disabledInit(self) -> None:
        self.driveTrain.coast()
        return super().disabledInit()
    
    def getNavxOneEighty(self):
        angle = self.navx.getAngle()
        angle %= 360
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        return angle
    
    def getNavx360(self):
        angle = self.navx.getAngle()
        angle %= 360
        return angle
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        return super().testPeriodic()
    
    def autonomousInit(self):
        pass
        

    def autonomousPeriodic(self):
        pass
        
        

    def teleopInit(self):
        self.driveTrain.fieldOrient = self.config["RobotDefaultSettings"]["fieldOrient"]
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        switches = self.driverStation.checkSwitches()
        # wpilib.SmartDashboard.putNumber("winch position", self.climber.rightArm.winch.getRotations())
        switches["driverX"], switches["driverY"], switches["driverZ"],  = self.evaluateDeadzones((switches["driverX"], switches["driverY"], switches["driverZ"]))
        if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
            self.driveTrain.move(switches["driverX"], switches["driverY"], switches["driverZ"])
        else:
            self.driveTrain.stationary()
        
    def disabledPeriodic(self):
        self.visionPeriodic()
    
    def disabledInit(self) -> None:
        self.driveTrain.coast()
    
    def switchActions(self, switches: dict):
        ''' Actually acts on and calls commands based on inputs from multiple robot modes '''
    
    def evaluateDeadzones(self, inputs):
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
        
    def visionPeriodic(self):
        '''result = mainCamera.getLatestResult()
        if result.hasTargets:
            target = result.getBestTarget()
            robotPose = target.getCameraRelativePose()
            targetX = robotPose.X()
            targetY = robotPose.Y()
            targetRotation = robotPose.rotation()
            wpilib.SmartDashboard.putString("targetX", str(targetX))
            wpilib.SmartDashboard.putString("targetY", str(targetY))
            wpilib.SmartDashboard.putString("targetRotation", str(targetRotation))
        else:
            wpilib.SmartDashboard.putString("targetX", "No Target")
            wpilib.SmartDashboard.putString("targetY", "No Target")
            wpilib.SmartDashboard.putString("targetRotation", "No Target")'''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)
