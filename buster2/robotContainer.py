import commands2
from commands2 import button, cmd
import wpilib
import os
import json
from wpimath import geometry

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class RobotContainer:
    PLACEHOLDER = geometry.Pose2d()
    
    folderPath = os.path.dirname(os.path.abspath(__file__))
    filePath = os.path.join(folderPath, 'config.json')
    with open (filePath, "r") as f1:
        config = json.load(f1)
    def __init__(self, MyRobot: commands2.TimedCommandRobot) -> None:
        # subsystems
        self.driveTrain = DriveTrainSubSystem(MyRobot, self.config)
        self.poseEstimator = PoseEstimatorSubsystem(MyRobot, self.driveTrain, self.PLACEHOLDER, self.config)
        
        # commands
        
        
        # buttons
        self.joystick = button.CommandJoystick(0)
        
        #subsystem configuration
        self.driveTrain.setDefaultCommand(cmd.run(lambda: self.driveTrain.joystickDrive(self.getJoystickInput()), [self.driveTrain]))
        self.configureButtonBindings()
    
    def configureButtonBindings(self):
        self.joystickButtonTwo = button.JoystickButton(self.joystick, 2)
        self.joystickButtonTwo.whileHeld(cmd.run(lambda: self.driveTrain.xMode(), [self.driveTrain]))
        
    def getJoystickInput(self):
        inputs = (self.joystick.getX(), self.joystick.getY(), self.joystick.getZ())
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
        return adjustedInputs[0], adjustedInputs[1], adjustedInputs[2]
    
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
    
    def getAutonomousCommand(self):
        return commands2.Command()
    
    def teleopInit(self):
        pass
    
    def disabledInit(self):
        self.driveTrain.coast()