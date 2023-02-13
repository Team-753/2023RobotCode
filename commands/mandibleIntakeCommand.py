from subsystems.mandible import MandibleSubSystem
import commands2
import wpilib

class MandibleIntakeCommand(commands2.CommandBase):
    ''' Command for setting the mandible to intake until it detects a game piece. '''
    
    def __init__(self, mandibleSubSystem: MandibleSubSystem) -> None:
        super().__init__()
        self.done = False
        self.addRequirements(mandibleSubSystem)
        self.mandible = mandibleSubSystem
        '''
        Depending on implementation, may need to add an extra timer-based failsafe for this
        '''
    
    def execute(self) -> None:
        if (self.mandible.inControlOfPiece()):
            self.done = True
        else:
            self.mandible.intake()
    
    def end(self, interrupted: bool) -> None:
        self.mandible.coast()
    
    def isFinished(self) -> bool:
        return self.done