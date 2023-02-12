import wpilib
import commands2

class AuxiliaryStreamDeckSubsystem(commands2.SubsystemBase):
    
    def __init__(self, USBPORT: int) -> None:
        super().__init__()
        self.streamDeckJoystick = wpilib.Joystick(USBPORT)
        self.results = []
    
    def periodic(self) -> None:
        switchList = []
        for i in range(10):
            switchList.append(self.streamDeckJoystick.getRawButton(i + 1))
        bindings = [
            "topLeft",
            "topMid",
            "topRight",
            "midLeft",
            "midMid",
            "midRight",
            "lowLeft",
            "lowMid",
            "lowRight"
        ]
        self.results = []
        for idx, boolean in enumerate(switchList):
            if boolean:
                self.results.append(bindings[idx])
    
    def getSwitches(self):
        pass # I have little use for this at the moment
    
    def getLastPressedSwitch(self) -> str:
        return self.results[0]