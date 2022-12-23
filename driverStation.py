import wpilib

class driverStation:
    def __init__(self, config: dict):
        self.config = config
        self.driverInput = wpilib.XboxController(0)
        self.auxiliaryInput = wpilib.XboxController(1)
    
    def checkSwitches(self):
        switches = {
            "driverX": 0.0,
            "driverY": 0.0,
            "driverZ": 0.0
        }
        switches["driverX"] = -self.driverInput.getLeftX()
        switches["driverY"] = self.driverInput.getLeftY()
        switches["driverZ"] = self.driverInput.getRightTriggerAxis() - self.driverInput.getLeftTriggerAxis()
        return switches