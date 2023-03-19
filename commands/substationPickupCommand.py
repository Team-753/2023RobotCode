import commands2
from commands2 import button
from wpimath import kinematics, controller, filter
import math
from typing import List

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class SubstationPickupCommand(commands2.CommandBase):
    ''' Keeps the robot at a static angle facing the substation whilst also locking the robot Y-value in place only giving the operator control of the forward/backward motion'''
    staticFrictionFFTurn = 0.15
    staticFrictionFFTranslate = 0.05
    targetAngle = 0 # facing away from the driverstation regardless of alliance
    angleTolerance = 1 # degrees
    translationTolerance = 0.01 # centimeters
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem, Joystick: button.CommandJoystick, Config: dict) -> None:
        super().__init__()
        self.addRequirements(DriveTrain)
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.joystick = Joystick
        self.config = Config
        self.angleController = controller.PIDController(1.5, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(math.radians(self.angleTolerance))
        self.strafeController = controller.PIDController(1, 0, 0, 0.05)
        self.strafeController.setTolerance(self.translationTolerance)
        self.kMaxSpeed = self.config["RobotDefaultSettings"]["wheelVelocityLimit"]
        
    def initialize(self) -> None:
        self.angleController.reset()
        self.strafeController.reset()
        self.targetYVal = self.poseEstimator.getCurrentPose().Y()
    
    def execute(self) -> None:
        joystickScalar = self.getJoystickInput()[1]
        currentPose = self.poseEstimator.getCurrentPose()
        rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.radians(self.targetAngle))
        if self.angleController.atSetpoint():
            rotationFeedback = 0
        if rotationFeedback < 0:
            rotFF = -self.staticFrictionFFTurn
        elif rotationFeedback > 0:
            rotFF = self.staticFrictionFFTurn
        else:
            rotFF = 0
        strafeFeedback = -self.strafeController.calculate(currentPose.Y(), self.targetYVal)
        if self.strafeController.atSetpoint():
            strafeFeedback = 0
        if strafeFeedback < 0:
            strafeFF = -self.staticFrictionFFTranslate
        elif strafeFeedback > 0:
            strafeFF = self.staticFrictionFFTranslate
        else:
            strafeFF = 0
        self.driveTrain.autoDrive(kinematics.ChassisSpeeds(joystickScalar * self.kMaxSpeed, strafeFeedback + strafeFF, rotationFeedback + rotFF), currentPose)
    
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