from networktables import NetworkTable
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from wpimath import controller, kinematics
from commands2 import button
import photonvision

import math
import commands2

'''class AutoAlignCommand(commands2.CommandBase):
    tolerance = 1 # +/- 1 degrees
    staticFrictionFFTurn = 0.2
    staticFrictionFFDrive = 0.075
    
    def __init__(self, PhotonCamera: photonvision.PhotonCamera, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem, Joystick: button.CommandJoystick, Config: dict, photonTable: NetworkTable) -> None:
        super().__init__()
        self.camera = PhotonCamera
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.txController = controller.PIDController(0.05, 0, 0, 0.05)
        self.txController.setTolerance(self.tolerance)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.joystick = Joystick
        self.config = Config
        self.photonTable = photonTable
        self.addRequirements(self.driveTrain)
        
    def initialize(self) -> None:
       #self.camera.setLEDMode(photonvision.LEDMode.kOn)
       self.photonTable.putNumber('ledMode', -1)
       
        
    def execute(self) -> None:
        result = self.camera.getLatestResult()
        if result.hasTargets(): # does the limelight have any valid targets?
            target = result.getBestTarget() # how far off we are from the target in degrees
            tx = target.getYaw()
            feedback = self.txController.calculate(tx, 0)
            currentPose = self.poseEstimator.getCurrentPose()
            rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi)
            if rotationFeedback < 0:
                rotFF = -self.staticFrictionFFTurn
            elif rotationFeedback > 0:
                rotFF = self.staticFrictionFFTurn
            else:
                rotFF = 0
            if feedback < 0:
                driveFF = -self.staticFrictionFFDrive
            elif feedback > 0:
                driveFF = self.staticFrictionFFDrive
            else:
                driveFF = 0
            self.driveTrain.autoDrive(kinematics.ChassisSpeeds(self.getJoystickInput()[1], feedback + driveFF, rotationFeedback + rotFF), currentPose)
        else:
            self.driveTrain.coast()
    
    def end(self, interrupted: bool) -> None:
        #self.camera.setLEDMode(photonvision.LEDMode.kOff) # sets it back to our apriltag pipeline
        self.photonTable.putNumber('ledMode', 0)
        self.driveTrain.coast()
        
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
        return adjustedInputs'''
    
class AutoAlignCommand(commands2.CommandBase):
    tolerance = 1 # +/- 1 degrees
    staticFrictionFFTurn = 0.2
    staticFrictionFFDrive = 0.075
    
    def __init__(self, LLTable: NetworkTable, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem, Joystick: button.CommandJoystick, Config: dict, photonTable: NetworkTable) -> None:
        super().__init__()
        self.llTable = LLTable
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.txController = controller.PIDController(0.05, 0, 0, 0.05)
        self.txController.setTolerance(self.tolerance)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.joystick = Joystick
        self.config = Config
        self.photonTable = photonTable
        self.addRequirements(self.driveTrain)
        
    def initialize(self) -> None:
       self.llTable.putNumber('pipeline', 1)
       self.llTable.putNumber('ledMode', 0)
       
        
    def execute(self) -> None:
        if self.llTable.getNumber('tv', 0) == 1: # does the limelight have any valid targets?
            tx = self.llTable.getNumber('tx', 0)
            feedback = self.txController.calculate(tx, 0)
            currentPose = self.poseEstimator.getCurrentPose()
            rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi)
            if rotationFeedback < 0:
                rotFF = -self.staticFrictionFFTurn
            elif rotationFeedback > 0:
                rotFF = self.staticFrictionFFTurn
            else:
                rotFF = 0
            if feedback < 0:
                driveFF = -self.staticFrictionFFDrive
            elif feedback > 0:
                driveFF = self.staticFrictionFFDrive
            else:
                driveFF = 0
            self.driveTrain.autoDrive(kinematics.ChassisSpeeds(self.getJoystickInput()[1], feedback + driveFF, rotationFeedback + rotFF), currentPose)
        else:
            self.driveTrain.coast()
    
    def end(self, interrupted: bool) -> None:
        self.llTable.putNumber('pipeline', 0)
        
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