import wpilib
import json
import os
import math
#from networktables import NetworkTables
import threading
import photonvision
from driveTrain import DriveTrain
from wpimath import geometry, controller, trajectory
import photonvision
import pathplannerlib
from poseEstimator import PoseEstimatorSubsystem
from wpilib import SmartDashboard

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
        SmartDashboard.putNumber("Target Rotation", 0)
        self.driveTrain = DriveTrain(self.config)
        self.joystick = wpilib.Joystick(0)
        self.estimator = PoseEstimatorSubsystem(self.driveTrain, geometry.Pose2d())
        xConstraints = trajectory.TrapezoidProfile.Constraints(3, 2)
        yConstraints = trajectory.TrapezoidProfile.Constraints(3, 2)
        zConstraints = trajectory.TrapezoidProfileRadians.Constraints(math.pi, math.pi)
        self.xPID = controller.PIDController(0.001, 0, 0)
        self.yPID = controller.PIDController(0.001, 0, 0)
        self.thetaPID = controller.ProfiledPIDControllerRadians(0.0075, 0.002, 0.003, zConstraints)
        self.thetaPID.enableContinuousInput(-math.pi, math.pi)
        self.swerveController = controller.HolonomicDriveController(self.xPID, self.yPID, self.thetaPID)
        pathName = "Rectangle"
        path = pathplannerlib.PathPlanner.loadPath(pathName, pathplannerlib.PathConstraints(4, 3), False)
        self.path = path.asWPILibTrajectory()
        self.driveTrain.reset()
        self.estimator.resetFieldPosition()
        self.driveTrain.navx.reset()
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        pass
    
    def testPeriodic(self) -> None:
        '''self.autonomous.followAprilTag()'''
        self.estimator.periodic()
        currentPose = self.estimator.getCurrentPose()
        SmartDashboard.putNumber("Robot Rotation", currentPose.rotation().degrees())
        goal = math.radians(SmartDashboard.getNumber("Target Rotation", 0))
        val = self.thetaPID.calculate(currentPose.rotation().radians(), goal)
        SmartDashboard.putNumber("rotation scalar", val)
        self.driveTrain.drive(0, 0, val)
    
    def autonomousInit(self):
        #self.driveTrain.reset()
        self.driveTrain.reset()
        #self.estimator.resetFieldPosition()
        self.estimator.resetFieldPosition()
        self.timer = wpilib.Timer()
        self.timer.start()
        self.autoDone = False
        
    def autonomousPeriodic(self):
        # self.autonomous.bigDaddy()
        self.estimator.periodic()
        time = self.timer.get()
        if time > self.path.totalTime():
            time = self.path.totalTime() - 0.02
        if (time < self.path.totalTime()):
            currentPose = self.estimator.getCurrentPose()
            SmartDashboard.putNumber("Robot X", currentPose.X())
            SmartDashboard.putNumber("Robot Y", currentPose.Y())
            SmartDashboard.putNumber("Robot Rotation", currentPose.rotation().degrees())
            goal = self.path.sample(time)
            chassisSpeeds = self.swerveController.calculate(currentPose, goal, geometry.Rotation2d(0))
            self.driveTrain.drive(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega)
        elif (self.autoDone == False):
            self.timer.stop()
            self.autoDone = True
            self.driveTrain.drive(0, 0, 0)
            print(f"Auto completed in: {self.timer.get()}s")
        else:
            self.driveTrain.drive(0, 0, 0)
        
    def teleopInit(self):
        self.driveTrain.navx.reset()
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        x, y, z = self.evaluateDeadzones([self.joystick.getX(), self.joystick.getY(), self.joystick.getZ()])
        self.driveTrain.joystickDrive(x, y, z)
    
    def evaluateDeadzones(self, inputs):
        '''This method takes in a list consisting of x input, y input, z input
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
        SmartDashboard.putNumber("Robot Rotation", self.estimator.getCurrentPose().rotation().degrees())
    
    def disabledInit(self) -> None:
        '''self.driveTrain.coast()'''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)