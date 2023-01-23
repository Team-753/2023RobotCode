import wpilib

class Operator:
    def __init__(self, config: dict) -> None:
        self.config = config
        self.joystick = wpilib.Joystick(self.config["driverStation"]["joystickUSB"])
        self.streamDeck = wpilib.Joystick(self.config["driverStation"]["streamDeckUSB"])
        
    def periodic(self):
        pass