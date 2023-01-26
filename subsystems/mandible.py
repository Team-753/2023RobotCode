import wpilib

class Mandible:
    ''''''
    state = "cube"
    
    def __init__(self) -> None:
        pass
    
    def doSomething(self, todo: str):
        pass
    
    def contract(self):
        ''' Pretty self-explanatory '''
        self.state = "cone"
        
    def release(self):
        ''' Pretty self-explanatory '''
        self.state = "cube"
        
    def intake(self):
        ''' NOTE: Needs to check whether or not game piece is fully in the mandible, distance sensor??? will return false until true. '''
        return bool
        
    def outtake(self):
        ''' Pretty self-explantory '''
        
    def coast(self):
        ''''''
    
    def brake(self):
        ''''''
    def getState(self):
        return self.state
    
    def setState(self, stateToSet: str):
        ''' Sets the desired mandible state, if the state is already set: do nothing.
        stateToSet: 'cone', 'cube' '''
        if stateToSet == "cone" and stateToSet != self.state: # we want a cone
            self.contract()
        elif stateToSet == "cube" and stateToSet != self.state: # we want a cube
            self.release()
    
    def setNeutralWheelState(self, wheelStateToSet: str):
        ''' Sets the state of the rotating wheels on the mandible.
        wheelStateToSet: 'intake', 'outtake',  '''