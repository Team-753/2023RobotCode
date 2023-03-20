from networktables import NetworkTable
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from wpimath import geometry, kinematics, controller

import commands2
import math
import wpilib

class LimeLightSanityCheck(commands2.CommandBase):
    tolerance = 1 # +/- 1 degrees
    staticFrictionFFTurn = 0.2
    
    def __init__(self, LLTable: NetworkTable, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem, Mandible: MandibleSubSystem) -> None:
        super().__init__()
        self.llTable = LLTable
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.mandible = Mandible
        self.txController = controller.PIDController(1, 0, 0, 0.05)
        self.txController.setTolerance(self.tolerance)
        
    def initialize(self) -> None:
        self.llTable.putNumber('pipeline', 0) # pipeline index one will be our detector pipeline
        self.finished = False
        self.txController.reset()
        
    def execute(self) -> None:
        if self.llTable.getNumber('tv', 0) == 1: # does the limelight have any valid targets?
            classID = self.llTable.getNumber('tclass', 0) # 0 is a cone, 1 is a cube
            if classID == 0:
                if self.mandible.state != 'Cone':
                    self.mandible.setState('Cone')
            else:
                if self.mandible.state != 'Cube':
                    self.mandible.setState('Cube')
            tx = self.llTable.getNumber('tx', 0) # how far off we are from the target in degrees
            if self.txController.atSetpoint():
                self.finished = True
            else:
                rotationFeedback = self.txController.calculate(tx, 0)
                if rotationFeedback < 0:
                    rotFF = -self.staticFrictionFFTurn
                else:
                    rotFF = self.staticFrictionFFTurn
                self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, 0, rotationFeedback + rotFF), self.poseEstimator.getCurrentPose())
    
    def end(self, interrupted: bool) -> None:
        self.llTable.putNumber('pipeline', 2) # sets it back to our apriltag pipeline
        self.driveTrain.stationary()
    
    def isFinished(self) -> bool:
        return self.finished
    
class TurnToPieceCommand(commands2.CommandBase):
    tolerance = 1 # +/- one degree
    staticFrictionFFTurn = 0.2
    FieldWidth = 16.54175
    FieldHeight = 8.0137
    gamePieceXValue = 7.11835 # in meters
    gamePieceYValues = [0.919226, 2.138426, 3.357626, 4.576826]
    verticalStack = [(gamePieceYValues[0] + gamePieceYValues[1]) / 2, (gamePieceYValues[1] + gamePieceYValues[2]) / 2, (gamePieceYValues[2] + gamePieceYValues[3]) / 2]
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.addRequirements(self.driveTrain)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        
    def initialize(self) -> None:
        currentPose = self.poseEstimator.getCurrentPose()
        self.done = False
        self.angleController.reset()
        self.targetAngle = self.calculateNearestPiece(currentPose)
    
    def execute(self) -> None:
        if self.angleController.atSetpoint():
            self.done = True
        else:
            currentPose = self.poseEstimator.getCurrentPose()
            rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), self.targetAngle.radians())
            if rotationFeedback < 0:
                rotFF = -self.staticFrictionFFTurn
            else:
                rotFF = self.staticFrictionFFTurn
            self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, 0, rotationFeedback + rotFF), currentPose)
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()
    
    def isFinished(self) -> bool:
        return self.done
    
    def calculateNearestPiece(self, currentPose: geometry.Pose2d) -> geometry.Rotation2d:
        currentYVal = currentPose.Y()
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            if currentYVal <= self.transformToAllianceYVal(self.verticalStack[2]):
                targetGamePieceY = self.transformToAllianceYVal(self.gamePieceYValues[3]) # bottom game piece
            elif currentYVal > self.transformToAllianceYVal(self.verticalStack[2]) and currentYVal <= self.transformToAllianceYVal(self.verticalStack[1]):
                targetGamePieceY = self.transformToAllianceYVal(self.gamePieceYValues[2]) # 2nd game piece from the bottom
            elif currentYVal > self.transformToAllianceYVal(self.verticalStack[1]) and currentYVal <= self.transformToAllianceYVal(self.verticalStack[0]):
                targetGamePieceY = self.transformToAllianceYVal(self.gamePieceYValues[1]) # 3rd game piece from the bottom
            else:
                targetGamePieceY = self.transformToAllianceYVal(self.gamePieceYValues[0]) # 4th game piece from the bottom
        else:
            if currentYVal <= self.verticalStack[0]:
                targetGamePieceY = self.gamePieceYValues[0] # bottom game piece
            elif currentYVal > self.verticalStack[0] and currentYVal <= self.verticalStack[1]:
                targetGamePieceY = self.gamePieceYValues[1] # 2nd game piece from the bottom
            elif currentYVal > self.verticalStack[1] and currentYVal <= self.verticalStack[2]:
                targetGamePieceY = self.gamePieceYValues[2] # 3rd game piece from the bottom
            else:
                targetGamePieceY = self.gamePieceYValues[3] # 4th game piece from the bottom
        xDiff = currentPose.X() - self.gamePieceXValue
        yDiff = currentPose.Y() - targetGamePieceY
        robotRotation = geometry.Rotation2d(xDiff, yDiff)
        return robotRotation

    def transformToAllianceYVal(self, value: float):
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            return self.FieldHeight - value
        return value

class DriveAndPickupPiece(commands2.CommandBase):
    
    def __init__(self, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem, Mandible: MandibleSubSystem, Arm: ArmSubSystem) -> None:
        super().__init__()
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.mandible = Mandible
        self.arm = Arm
        
    def initialize(self) -> None:
        self.mandible.intake()
    
    def execute(self) -> None: # may need to implement a heading control PID here
        self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0.5, 0, 0), self.poseEstimator.getCurrentPose(), False)
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()
        self.arm.setPosition('Optimized')
    
    def isFinished(self) -> bool:
        return self.poseEstimator.getCurrentPose().X() > 7.11835 or self.mandible.inControlOfPiece()