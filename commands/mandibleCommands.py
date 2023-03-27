from subsystems.mandible import MandibleSubSystem
import commands2
import wpilib

class MandibleOuttakeCommand(commands2.CommandBase):
    ''''''
    
    def __init__(self, mandibleSubSystem: MandibleSubSystem) -> None:
        super().__init__()
        self.addRequirements(mandibleSubSystem)
        self.mandible = mandibleSubSystem
        self.timer = wpilib.Timer()
        self.wait = 0.75
        '''
        Depending on implementation, may need to add an extra timer-based failsafe for this
        '''
    
    def initialize(self) -> None:
        self.timer.restart()
    
    def execute(self) -> None:
        self.mandible.outtake()
    
    def end(self, interrupted: bool) -> None:
        self.mandible.stop()
        self.timer.stop()
        
    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.wait) or self.mandible.state == 'Cone'

class MandibleIntakeCommand(commands2.CommandBase):
    ''' Command for setting the mandible to intake until it detects a game piece. '''
    
    def __init__(self, mandibleSubSystem: MandibleSubSystem) -> None:
        super().__init__()
        self.addRequirements(mandibleSubSystem)
        self.mandible = mandibleSubSystem
        '''
        Depending on implementation, may need to add an extra timer-based failsafe for this
        '''
    def initialize(self) -> None:
        self.done = False
        return super().initialize()
    
    def execute(self) -> None:
        self.mandible.intake() # fix this for auto
    
    def end(self, interrupted: bool) -> None:
        self.mandible.stop()
    
    def isFinished(self) -> bool:
        return self.done