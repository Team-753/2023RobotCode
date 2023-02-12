import commands2
from commands2 import cmd, button

import pathplannerlib

from commands.substationPickupCommand import SubstationPickupCommand
from commands.placeOnGridCommand import PlaceOnGridCommand

from auto.swerveAutoBuilder import SwerveAutoBuilder

from subsystems.arm import ArmSubSystem
from subsystems.mandible import MandibleSubSystem
from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class UniversalMacroCommand(commands2.CommandBase):
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Constraints: pathplannerlib.PathConstraints, EventMap: dict, DriverCommandJoystick: button.CommandJoystick, DriveTrain: DriveTrainSubSystem) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm, DriveTrain])
        
    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        return super().execute()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()