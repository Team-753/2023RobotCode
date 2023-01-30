import wpilib
import ctre
import commands2

class ArmSubSystem(commands2.SubsystemBase):
    ''''''
    
    constants = { # what encoder value the arm's lead screw motor should go to according to the needed position
        "fullyRetracted": 0.0,
        "substation": 0.0,
        "floor": 0.0,
        "highCone": 0.0,
        "midCone": 0.0,
        "highCube": 0.0,
        "midCube": 0.0
    }
    targetValue = 0
    
    def __init__(self) -> None:
        super().__init__()

    def periodic(self) -> None:
        ''' Runs every 20ms in all modes, keep that in mind Joe. '''
        '''
        TODO:
        - Implement Position PID controller on whatever motor we end up going with, if falcon 500 internal PID is feasible though we may
        need to switch over PID's based on how far out the arm is extended.
        - Returns: whether it is within the setpoint threshold, very important for auto stuff
        '''
        return super().periodic()

    def setPosition(self, position: str):
        self.targetValue = self.constants[position]
        
    def coast(self):
        ''''''
        
    def brake(self):
        ''''''
        
    def zero(self):
        ''' Returns the arm to the starting positon, hitting a limit switch and therefore finding the relative zero '''
        
    def manualControl(self, direction: str):
        ''' As the function name says, will need some kind of linear/nonlinear scalar value as the arm speed is very non linear. '''
        # TODO: Implement control by changing position over time, think an incrementer every 20ms