from subsystems.arm import ArmSubSystem
import commands2
import wpilib

class ArmConfirmPlacementCommand(commands2.CommandBase):
    ''' Makes the arm go to a given position and won't finish until the arm is within position tolerance '''
    
    def __init__(self, armSubSystem: ArmSubSystem, targetPositionString: str) -> None:
        super().__init__()
        self.addRequirements(armSubSystem)
        self.arm = armSubSystem
        self.target = targetPositionString
        '''
        Depending on implementation, may need to add an extra timer-based failsafe for this
        '''
    
    def initialize(self) -> None:
        self.arm.setPosition(self.target)
    
    def isFinished(self) -> bool:
        return self.arm.atSetpoint()