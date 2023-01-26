import wpilib
import ctre

class Arm:
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
    
    def __init__(self) -> None:
        ''' Left unused for now '''
    
    def goToPosition(self, position: str):
        targetValue = self.constants[position]
        '''
        TODO:
        - Implement Position PID controller on whatever motor we end up going with, if falcon 500 internal PID is feasible though we may
        need to switch over PID's based on how far out the arm is extended.
        - Returns: whether it is within the setpoint threshold, very important for auto stuff
        '''
        
    def coast(self):
        ''''''
        
    def brake(self):
        ''''''
        
    def zero(self):
        ''' Returns the arm to the starting positon, hitting a limit switch and therefore finding the relative zero '''
        
    def manualControl(self):
        ''' As the function name says, will need some kind of linear/nonlinear scalar value as the arm speed is very non linear. '''