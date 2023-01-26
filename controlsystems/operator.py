import wpilib
from wpilib import shuffleboard

class Operator:
    counter = 0
    blank = {
        "driverX": 0.0,
        "driverY": 0.0,
        "driverZ": 0.0,
        "placeTopRight": False,
        "placeTopMid": False,
        "placeTopLeft": False,
        "placeMidRight": False,
        "placeMidMid": False,
        "placeMidLeft": False,
        "placeBottomRight": False,
        "placeBottomMid": False,
        "placeBottomLeft": False
    }
    driverTab = shuffleboard.Shuffleboard.getTab("Driver")
    
    def __init__(self, config: dict) -> None:
        self.config = config
        self.driverTab.add("Joystick Connected", False).withPosition(12, 0)
        self.driverTab.add("Stream Deck Connected", False).withPosition(12, 1)
        self.joystick = wpilib.Joystick(self.config["driverStation"]["joystickUSB"])
        self.streamDeck = wpilib.Joystick(self.config["driverStation"]["streamDeckUSB"])
        self.checkInputDevices()
        
    
    def checkInputDevices(self):
        '''if (self.counter == 50):
            self.counter = 0
            if (self.joystick.getName() != "Logitech Extreme 3D" and self.driverTab):
                self.driverTab.add("Joystick Connected", False).withPosition(12, 0)
            else:
                self.driverTab.add("Joystick Connected", True).withPosition(12, 0)
            if (self.streamDeck.getName() != "vJoy Device"):
                self.driverTab.add("Stream Deck Connected", False).withPosition(12, 1)
            else:
                self.driverTab.add("Stream Deck Connected", True).withPosition(12, 1)
        else:
            self.counter += 1'''
                
        
    def periodic(self):
        self.checkInputDevices()
        switches = self.blank.copy()