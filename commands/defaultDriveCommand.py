import commands2
from commands2 import button
from math import hypot
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from wpimath import filter
from typing import List

class DefaultDriveCommand(commands2.CommandBase):
    
    def __init__(self, Joystick: button.CommandJoystick, DriveTrainSubSystem: DriveTrainSubSystem, PoseEstimatorSubSystem: PoseEstimatorSubsystem, config: dict) -> None:
        super().__init__()
        self.addRequirements(DriveTrainSubSystem) # no need to require the pose estimator subsystem
        self.joystick = Joystick
        self.driveTrain = DriveTrainSubSystem
        self.poseEstimator = PoseEstimatorSubSystem
        self.config = config
        self.kMaxSpeed = self.config["RobotDefaultSettings"]["wheelVelocityLimit"]
        self.maxAngularVelocity = self.config["driverStation"]["teleoperatedRobotConstants"]["teleopVelLimit"] / hypot(self.config["RobotDimensions"]["trackWidth"] / 2, self.config["RobotDimensions"]["wheelBase"] / 2) # about 11 rads per second
        
    def execute(self) -> None:
        inputs = self.getJoystickInput()
        self.driveTrain.joystickDrive([inputs[1] * self.kMaxSpeed, inputs[0] * self.kMaxSpeed, inputs[2] * self.maxAngularVelocity], self.poseEstimator.getCurrentPose())
    
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
        return adjustedInputs