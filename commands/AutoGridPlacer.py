import commands2
import math
import pathplannerlib
import wpilib
import photonvision

from commands2 import cmd, button
from wpimath import geometry, kinematics, controller
from typing import List, Tuple

from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.streamDeck import StreamDeckSubsystem

from commands.armConfirmPlacementCommand import ArmConfirmPlacementCommand
from commands.mandibleCommands import MandibleOuttakeCommand
from networktables import NetworkTable


class AutoGridPlacer:
    placementSequenceCounter = 0
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, 
                 Mandible: MandibleSubSystem, 
                 Arm: ArmSubSystem, 
                 PoseEstimator: PoseEstimatorSubsystem, 
                 DriveTrain: DriveTrainSubSystem, 
                 Constraints: pathplannerlib.PathConstraints, 
                 DriverJoystick: button.CommandJoystick, 
                 StreamDeck: StreamDeckSubsystem, 
                 config: dict, 
                 LimelightTable: NetworkTable,
                 PlacementSpots: List[wpilib.SendableChooser]) -> None:
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.driveTrain = DriveTrain
        self.constraints = Constraints
        self.joystick = DriverJoystick
        self.streamDeck = StreamDeck
        self.config = config
        self.placementSpots = PlacementSpots
        self.llTable = LimelightTable
        self.stageController = StageController(self.arm, self.mandible, self.driveTrain, self.swerveAutoBuilder, self.poseEstimator)
    
    def reset(self) -> None:
        self.placementSequenceCounter = 0

    def getCommandSequence(self, isAutonomous = False) -> commands2.Command:
        if isAutonomous:
            selectedSlot = self.placementSpots[self.placementSequenceCounter].getSelected()
            self.placementSequenceCounter += 1
            self.stageController.setTarget(selectedSlot, self.poseEstimator.getCurrentPose())
            if selectedSlot[1] == 1 or selectedSlot[1] == 4 or selectedSlot[1] == 7: # it is a cube, we don't need all the extra stuff
                return commands2.SequentialCommandGroup(self.stageController.calculateArmPosition(), self.stageController.calculateInitialPPCommand(), self.stageController.calculateFinalSequence())
            else:
                return commands2.SequentialCommandGroup(self.stageController.calculateArmPosition(), self.stageController.calculateInitialPPCommand(), LimeLightSanityCheck(self.llTable, self.driveTrain, self.poseEstimator), self.stageController.calculateFinalPPCommand(), self.stageController.calculateFinalSequence(), self.stageController.calculateAutoPPCommand())
        else:
            selectedSlot = self.streamDeck.getSelectedGridSlot()
            self.stageController.setTarget(selectedSlot)
            if selectedSlot[1] == 1 or selectedSlot[1] == 4 or selectedSlot[1] == 7: # it is a cube, we don't need all the extra stuff
                return commands2.SequentialCommandGroup(commands2.PrintCommand("starting sequence"), self.stageController.calculateInitialPPCommand(), commands2.PrintCommand("initial path following done"), cmd.runOnce(lambda: self.driveTrain.stationary(), [self.driveTrain]), commands2.PrintCommand("commencing final sequence"), self.stageController.calculateFinalSequence())
            else:
                return commands2.SequentialCommandGroup(self.stageController.calculateArmPosition(), self.stageController.calculateInitialPPCommand(), LimeLightSanityCheck(self.llTable, self.driveTrain, self.poseEstimator), self.stageController.calculateFinalPPCommand(), DriverConfirmCommand(self.joystick, self.driveTrain), self.stageController.calculateFinalSequence())

class StageController:
    GridLayout = [
        [
            [0.4191, 4.987417], 
            [0.4191, 4.424426], 
            [0.4191, 3.861435], 
            [0.8509, 4.987417], 
            [0.8509, 4.424426], 
            [0.8509, 3.861435], 
            [1.27635, 4.987417], 
            [1.27635, 4.424426], 
            [1.27635, 3.861435]
            ], 
        [
            [0.4191, 3.311017], 
            [0.4191, 2.748026], 
            [0.4191, 2.185035], 
            [0.8509, 3.311017], 
            [0.8509, 2.748026], 
            [0.8509, 2.185035], 
            [1.27635, 3.311017], 
            [1.27635, 2.748026], 
            [1.27635, 2.185035]
            ], 
        [
            [0.4191, 1.634617], 
            [0.4191, 1.071626], 
            [0.4191, 0.508635], 
            [0.8509, 1.634617], 
            [0.8509, 1.071626], 
            [0.8509, 0.508635], 
            [1.27635, 1.634617], 
            [1.27635, 1.071626], 
            [1.27635, 0.508635]
            ]
        ]
    FieldWidth = 16.54175
    FieldHeight = 8.0137
    highConeOffset = 1.397
    highCubeOffset = 1.7018
    midConeOffset = 1.45
    midCubeOffset = 1.7018
    lowConeOffset = 1.5494
    lowCubeOffset = 1.5494
    preOffset = 0.15 # 15cm
    targetSlot = (1, 4)
    autoPose = geometry.Pose2d()
    onTheFlyPathConstraints = pathplannerlib.PathConstraints(maxVel = 0.5, maxAccel = 1)

    def __init__(self, Arm: ArmSubSystem, Mandible: MandibleSubSystem, DriveTrain: DriveTrainSubSystem, SwerveAutoBuilder: SwerveAutoBuilder, PoseEstimator: PoseEstimatorSubsystem) -> None:
        self.arm = Arm
        self.mandible = Mandible
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.poseEstimator = PoseEstimator
        self.driveTrain = DriveTrain
    
    def setTarget(self, target: tuple, AutoPose = geometry.Rotation2d()) -> None:
        self.targetSlot = target
        self.autoPose = AutoPose
    
    def calculateArmPosition(self) -> commands2.Command:
        slot = self.targetSlot[1]
        if slot < 3: # we are placing high
            if slot == 1: # we are placing high cube
                return cmd.runOnce(lambda: self.arm.setPosition('HighCube'), [])
            else:
                return cmd.runOnce(lambda: self.arm.setPosition('HighConePrep'), [])
        elif slot > 5: # we are placing low
            return cmd.runOnce(lambda: self.arm.setPosition('BottomPlacement'), [])
        else: # we are placing mid
            if slot == 4: # we are placing mid cube
                return cmd.runOnce(lambda: self.arm.setPosition('MidCube'), [])
            else: # we are placing mid cone
                return cmd.runOnce(lambda: self.arm.setPosition('MidConePrep'), [])
            
    def calculateFinalSequence(self) -> commands2.Command:
        slot = self.targetSlot[1]
        if slot < 3: # we are placing high
            if slot == 1: # we are placing high cube
                return commands2.SequentialCommandGroup(ArmConfirmPlacementCommand(self.arm, 'HighCube'), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition('Optimized'), []))
            else:
                return commands2.SequentialCommandGroup(ArmConfirmPlacementCommand(self.arm, 'HighConePlacement'), cmd.runOnce(lambda: self.mandible.setState('Cube'), []), cmd.runOnce(lambda: self.arm.setPosition('Optimized'), []), commands2.WaitCommand(0.25), cmd.runOnce(lambda: self.mandible.setState('Cone')))
        elif slot > 5: # we are placing low
            return commands2.SequentialCommandGroup(ArmConfirmPlacementCommand(self.arm, 'BottomPlacement'), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition('Optimized'), []))
        else: # we are placing mid
            if slot == 4: # we are placing mid cube
                return commands2.SequentialCommandGroup(ArmConfirmPlacementCommand(self.arm, 'MidCube'), MandibleOuttakeCommand(self.mandible), cmd.runOnce(lambda: self.arm.setPosition('Optimized'), []))
            else: # we are placing mid cone
                return commands2.SequentialCommandGroup(ArmConfirmPlacementCommand(self.arm, 'MidConePlacement'), cmd.runOnce(lambda: self.mandible.setState('Cube'), []), cmd.runOnce(lambda: self.arm.setPosition('Optimized'), []), commands2.WaitCommand(0.25), cmd.runOnce(lambda: self.mandible.setState('Cone')))
    
    def calculateInitialPPCommand(self) -> commands2.Command:
        slotData = self.GridLayout[self.targetSlot[0]][self.targetSlot[1]]
        x = slotData[0]
        y = slotData[1]
        slot = self.targetSlot[1]
        if slot < 3: # we are placing high
            if slot == 1: # we are placing high cube
                offset = self.highCubeOffset - self.preOffset
            else:
                offset = self.highConeOffset
        elif slot > 5: # we are placing low
            offset = self.lowCubeOffset - self.preOffset
        else: # we are placing mid
            if slot == 4: # we are placing mid cube
                offset = self.midCubeOffset - self.preOffset
            else: # we are placing mid cone
                offset = self.midConeOffset
        targetPose = geometry.Pose2d(geometry.Translation2d(x = x + offset + self.preOffset, y = y), geometry.Rotation2d(math.pi))
        #print("**************************************************************************************\n**********************************************************************************************\n**********************************")
        wpilib.SmartDashboard.putString("target pose", f"X: {targetPose.X()}, Y: {targetPose.Y()}, Z: {targetPose.rotation().degrees()}")
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            targetPose = self.flipPoseToRedAlliance(targetPose)
        return self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.onTheFlyPathConstraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.poseEstimator.getCurrentPose(), self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(targetPose, kinematics.ChassisSpeeds(0, 0, 0))]))
    
    def calculateFinalPPCommand(self) -> commands2.Command:
        currentPose = self.poseEstimator.getCurrentPose() # so where we are now should be aligned on the cone thingy
        targetPose = geometry.Pose2d(currentPose.X() - self.preOffset, currentPose.Y(), geometry.Rotation2d(math.pi))
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            targetPose = self.flipPoseToRedAlliance(targetPose)
        return self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.onTheFlyPathConstraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint(targetPose.translation(), targetPose.rotation(), targetPose.rotation())]))
    
    def calculateAutoPPCommand(self) -> commands2.Command:
        currentPose = self.poseEstimator.getCurrentPose()
        return self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.onTheFlyPathConstraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(self.autoPose, kinematics.ChassisSpeeds(0, 0, 0))]))
    
    def flipPoseToRedAlliance(self, poseToFlip: geometry.Pose2d):
        return geometry.Pose2d(poseToFlip.X(), self.FieldHeight - poseToFlip.Y(), poseToFlip.rotation())
    
class LimeLightSanityCheck(commands2.CommandBase):
    tolerance = 1 # +/- 1 degrees
    staticFrictionFFTurn = 0.2
    staticFrictionFFDrive = 0.2
    
    def __init__(self, LLTable: NetworkTable, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.llTable = LLTable
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.txController = controller.PIDController(0.1, 0, 0, 0.05)
        self.txController.setTolerance(self.tolerance)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.timeoutCounter = wpilib.Timer()
        
    def initialize(self) -> None:
        self.llTable.putNumber('pipeline', 1) # pipeline index one will be our reflective tape pipeline
        self.finished = False
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.isRedAlliance = True
        else:
            self.isRedAlliance = False
        self.txController.reset()
        self.angleController.reset()
        self.timeoutCounter.reset()
        
    def execute(self) -> None:
        if self.llTable.getNumber('tv', 0) == 1: # does the limelight have any valid targets?
            self.timeoutCounter.stop()
            tx = self.llTable.getNumber('tx', 0) # how far off we are from the target in degrees
            feedback = self.txController.calculate(tx, 0)
            if self.txController.atSetpoint() and self.angleController.atSetpoint():
                self.finished = True
            else:
                currentPose = self.poseEstimator.getCurrentPose()
                rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi)
                if rotationFeedback < 0:
                    rotFF = -self.staticFrictionFFTurn
                else:
                    rotFF = self.staticFrictionFFTurn
                if feedback < 0:
                    driveFF = -self.staticFrictionFFDrive
                else:
                    driveFF = self.staticFrictionFFDrive
                self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, feedback + driveFF, rotationFeedback + rotFF), currentPose)
        else:
            self.timeoutCounter.start()
            if self.timeoutCounter.hasElapsed(2):
                self.finished = True
    
    def end(self, interrupted: bool) -> None:
        self.llTable.putNumber('pipeline', 0) # sets it back to our apriltag pipeline
        self.driveTrain.stationary()
    
    def isFinished(self) -> bool:
        return self.finished

class DriverConfirmCommand(commands2.CommandBase):
    
    def __init__(self, Joystick: button.CommandJoystick, DriveTrain: DriveTrainSubSystem) -> None:
        super().__init__()
        self.joystick = Joystick
        self.driveTrain = DriveTrain
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        return super().execute()
    
    def isFinished(self) -> bool:
        if self.joystick.getRawButtonPressed(1):
            return True
        else:
            return False
    
    def end(self, interrupted: bool) -> None:
        self.driveTrain.stationary()

class PhotonCameraSanityCheck(commands2.CommandBase):
    tolerance = 1 # +/- 1 degrees
    staticFrictionFFTurn = 0.2
    staticFrictionFFDrive = 0.2
    
    def __init__(self, Camera: photonvision.PhotonCamera, DriveTrain: DriveTrainSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.camera = Camera
        self.driveTrain = DriveTrain
        self.poseEstimator = PoseEstimator
        self.txController = controller.PIDController(0.1, 0, 0, 0.05)
        self.txController.setTolerance(self.tolerance)
        self.angleController = controller.PIDController(1, 0, 0, 0.05)
        self.angleController.enableContinuousInput(-math.pi, math.pi)
        self.angleController.setTolerance(self.tolerance)
        self.timeoutCounter = wpilib.Timer()
        
    def initialize(self) -> None:
        self.camera.setLEDMode(photonvision.LEDMode.kOn)
        self.finished = False
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.isRedAlliance = True
        else:
            self.isRedAlliance = False
        self.txController.reset()
        self.angleController.reset()
        self.timeoutCounter.reset()
        
    def execute(self) -> None:
        result = self.camera.getLatestResult()
        if result.hasTargets(): # does the photoncamera have any valid targets?
            self.timeoutCounter.stop()
            currentPose = self.poseEstimator.getCurrentPose()
            target = result.getBestTarget()
            tx = -target.getYaw() # how far off we are from the target in degrees
            feedback = self.txController.calculate(tx, 0)
            rotationFeedback = self.angleController.calculate(currentPose.rotation().radians(), math.pi)
            if self.txController.atSetpoint() and self.angleController.atSetpoint():
                self.finished = True
            else:
                if rotationFeedback < 0:
                    rotFF = -self.staticFrictionFFTurn
                else:
                    rotFF = self.staticFrictionFFTurn
                if feedback < 0:
                    driveFF = -self.staticFrictionFFDrive
                else:
                    driveFF = self.staticFrictionFFDrive
                self.driveTrain.autoDrive(kinematics.ChassisSpeeds(0, feedback + driveFF, rotationFeedback + rotFF), currentPose)
        else:
            self.timeoutCounter.start()
            if self.timeoutCounter.hasElapsed(2):
                self.finished = True
    
    def end(self, interrupted: bool) -> None:
        self.camera.setLEDMode(photonvision.LEDMode.kOff) # sets it back to our apriltag pipeline
        self.driveTrain.stationary()
    
    def isFinished(self) -> bool:
        return self.finished