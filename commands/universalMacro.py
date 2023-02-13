import commands2
from commands2 import cmd, button
from wpilib import DriverStation

import pathplannerlib

from commands.substationPickupCommand import SubstationPickupSubCommand
from commands.placeOnGridCommand import PlaceOnGridCommand

from auto.swerveAutoBuilder import SwerveAutoBuilder

from subsystems.arm import ArmSubSystem
from subsystems.mandible import MandibleSubSystem
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class UniversalMacroCommand(commands2.CommandBase):
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Constraints: pathplannerlib.PathConstraints, EventMap: dict, DriverCommandJoystick: button.CommandJoystick, DriveTrain: DriveTrainSubSystem, AuxiliaryStreamDeck: button.CommandJoystick) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm, DriveTrain])
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.constraints = Constraints
        self.eventMap = EventMap
        self.driveTrain = DriveTrain
        self.driverCommandJoystick = DriverCommandJoystick
        self.auxiliaryStreamDeck = AuxiliaryStreamDeck
        self.mode = None
        currentRobotPose = self.poseEstimator.getCurrentPose()
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if currentRobotPose.X() > 8.27: # we are closer to the substation than the grids, this is probably the command we want
                self.mode = "substationPickup"
            else: # We are closer to the grid than the substation, we probably want to try and place something
                self.mode = "gridPlacement"
        elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if currentRobotPose.X() < 8.27: # we are closer to the substation than the grids, this is probably the command we want
                self.mode = "substationPickup"
            else: # We are closer to the grid than the substation, we probably want to try and place something
                self.mode = "gridPlacement"
        if self.mode == "substationPickup":
            self.substationPickupCommand = SubstationPickupSubCommand(self.swerveAutoBuilder, self.mandible, self.arm, self.poseEstimator, self.constraints)
            
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        return super().execute()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()