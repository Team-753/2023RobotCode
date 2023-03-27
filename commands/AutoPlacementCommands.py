from wpimath import geometry, kinematics
import commands2
import math
import wpilib
from commands2 import button
from subsystems.streamDeck import StreamDeckSubsystem

class Calculator:
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
    
    def __init__(self, StreamDeck: StreamDeckSubsystem) -> None:
        self.streamDeck = StreamDeck
        self.targetSlot = (1, 4)
    
    def reInitialize(self) -> None:
        self.targetSlot = self.streamDeck.getSelectedGridSlot()
    
    def getInitialArmPosition(self) -> str:
        slot = self.targetSlot[1]
        if slot < 3: # we are placing high
            if slot == 1: # we are placing high cube
                return 'HighCube'
            else:
                return 'HighConePrep'
        elif slot > 5: # we are placing low
            return 'BottomPlacement'
        else: # we are placing mid
            if slot == 4: # we are placing mid cube
                return 'MidCube'
            else: # we are placing mid cone
                return 'MidConePrep'
            
    def calculateRobotPose(self) -> geometry.Pose2d:
        slot = self.targetSlot[1]
        slotData = self.GridLayout[self.targetSlot[0]][slot]
        x = slotData[0]
        y = slotData[1]
        slot = self.targetSlot[1]
        if slot < 3: # we are placing high
            if slot == 1: # we are placing high cube
                offset = self.highCubeOffset
            else:
                offset = self.highConeOffset
        elif slot > 5: # we are placing low
            offset = self.lowCubeOffset
        else: # we are placing mid
            if slot == 4: # we are placing mid cube
                offset = self.midCubeOffset
            else: # we are placing mid cone
                offset = self.midConeOffset
        targetPose = geometry.Pose2d(geometry.Translation2d(x = x + offset, y = y), geometry.Rotation2d(math.pi))
        #wpilib.SmartDashboard.putString("target pose", f"X: {targetPose.X()}, Y: {targetPose.Y()}, Z: {targetPose.rotation().degrees()}")
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            targetPose = self.flipPoseToRedAlliance(targetPose)
        return targetPose
    
    def getFinalArmPosition(self) -> str:
        slot = self.targetSlot[1]
        if slot < 3: # we are placing high
            if slot == 1: # we are placing high cube
                return 'HighCube'
            else:
                return 'HighConePlacement'
        elif slot > 5: # we are placing low
            return 'BottomPlacement'
        else: # we are placing mid
            if slot == 4: # we are placing mid cube
                return 'MidCube'
            else: # we are placing mid cone
                return 'MidConePlacement'
            
    def flipPoseToRedAlliance(self, poseToFlip: geometry.Pose2d):
        return geometry.Pose2d(poseToFlip.X(), self.FieldHeight - poseToFlip.Y(), poseToFlip.rotation())
    
class DriverConfirmCommand(commands2.CommandBase):
    
    def __init__(self, Joystick: button.CommandJoystick) -> None:
        super().__init__()
        self.joystick = Joystick
    
    def isFinished(self) -> bool:
        if self.joystick.getRawButtonPressed(1):
            return True
        else:
            return False