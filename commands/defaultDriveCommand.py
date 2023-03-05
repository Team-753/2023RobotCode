import commands2
from commands2 import button
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from wpilib import SmartDashboard
from wpimath import geometry
from math import radians

class DefaultDriveCommand(commands2.CommandBase):
    
    def __init__(self, Joystick: button.CommandJoystick, DriveTrainSubSystem: DriveTrainSubSystem, PoseEstimatorSubSystem: PoseEstimatorSubsystem, config: dict) -> None:
        super().__init__()
        self.addRequirements(DriveTrainSubSystem) # no need to require the pose estimator subsystem
        self.joystick = Joystick
        self.driveTrain = DriveTrainSubSystem
        self.poseEstimator = PoseEstimatorSubSystem
        self.config = config
        
    def execute(self) -> None:
        self.driveTrain.joystickDrive(self.getJoystickInput(), self.poseEstimator.getCurrentPose())
    
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