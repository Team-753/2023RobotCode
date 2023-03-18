import commands2
from commands2 import button
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from wpimath import filter
from typing import List

class DefaultDriveCommand(commands2.CommandBase):
    
    def __init__(self, Joystick: button.CommandJoystick, DriveTrainSubSystem: DriveTrainSubSystem, PoseEstimatorSubSystem: PoseEstimatorSubsystem, config: dict, RateLimiters: List[filter.SlewRateLimiter]) -> None:
        super().__init__()
        self.addRequirements(DriveTrainSubSystem) # no need to require the pose estimator subsystem
        self.joystick = Joystick
        self.driveTrain = DriveTrainSubSystem
        self.poseEstimator = PoseEstimatorSubSystem
        self.config = config
        self.rateLimiters = RateLimiters
        
    def execute(self) -> None:
        inputs = self.getJoystickInput()
        self.driveTrain.joystickDrive([self.rateLimiters[0].calculate(inputs[0]), self.rateLimiters[1].calculate(inputs[1]), self.rateLimiters[2].calculate(inputs[2])], self.poseEstimator.getCurrentPose())
    
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