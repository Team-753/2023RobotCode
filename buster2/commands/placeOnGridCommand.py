import pathplannerlib
import commands2
from commands2 import cmd, button

# For intellisense
from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.auxiliaryStreamDeck import AuxiliaryStreamDeckSubsystem

class PlaceOnGridCommand(commands2.CommandBase):
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Constraints: pathplannerlib.PathConstraints, EventMap: dict, DriverCommandJoystick: button.CommandJoystick, auxiliaryStreamDeck: AuxiliaryStreamDeckSubsystem) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm])
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.eventMap = EventMap
        self.driverCommandJoystick = DriverCommandJoystick
        self.constraints = Constraints
        self.auxiliaryStreamDeck = auxiliaryStreamDeck
        
    def initialize(self) -> None:
        self.currentPose = self.poseEstimator.getCurrentPose()
    
    def execute(self) -> None:
        return super().execute()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()