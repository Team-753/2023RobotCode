import commands2
from commands2 import button, cmd
from subsystems.arm import ArmSubSystem
import wpilib


class StreamDeckSubsystem(commands2.SubsystemBase):
    
    def __init__(self, ArmSubSystem: ArmSubSystem, StreamDeck: button.CommandJoystick) -> None:
        super().__init__()
        self.arm = ArmSubSystem
        self.streamDeck = StreamDeck
        self.selectedGridSlot = (2, 5)
        
    def periodic(self) -> None:
        switches = {
            "1": self.streamDeck.getRawButtonReleased(1),
            "2": self.streamDeck.getRawButtonReleased(2),
            "3": self.streamDeck.getRawButtonReleased(3),
            "4": self.streamDeck.getRawButtonReleased(4),
            "5": self.streamDeck.getRawButtonReleased(5),
            "6": self.streamDeck.getRawButtonReleased(6),
            "7": self.streamDeck.getRawButtonReleased(7),
            "8": self.streamDeck.getRawButtonReleased(8),
            "9": self.streamDeck.getRawButtonReleased(9),
            "GridOne": self.streamDeck.getRawButtonReleased(10),
            "GridTwo": self.streamDeck.getRawButtonReleased(11),
            "GridThree": self.streamDeck.getRawButtonReleased(12),
            "FullyRetracted": self.streamDeck.getRawButtonReleased(13),
            "Optimized": self.streamDeck.getRawButtonReleased(14),
            "Substation": self.streamDeck.getRawButtonReleased(15),
            "Floor": self.streamDeck.getRawButtonReleased(16),
            "BottomSlot": self.streamDeck.getRawButtonReleased(17),
            "MidCone": self.streamDeck.getRawButtonReleased(18),
            "MidCube": self.streamDeck.getRawButtonReleased(19),
            "HighCone": self.streamDeck.getRawButtonReleased(20),
            "HighCube": self.streamDeck.getRawButtonReleased(21),
        }
        for i in range(9):
            if switches[f"{i + 1}"]:
                self.selectedGridSlot = (self.selectedGridSlot[0], i + 1)
                wpilib.SmartDashboard.putNumber("Selected Grid Slot", i + 1)
        if switches["GridOne"]:
            self.selectedGridSlot = (1, self.selectedGridSlot[1])
            wpilib.SmartDashboard.putNumber("Selected Grid", 1)
        elif switches["GridTwo"]:
            self.selectedGridSlot = (2, self.selectedGridSlot[1])
            wpilib.SmartDashboard.putNumber("Selected Grid", 2)
        elif switches["GridThree"]:
            self.selectedGridSlot = (3, self.selectedGridSlot[1])
            wpilib.SmartDashboard.putNumber("Selected Grid", 3)
        if switches["FullyRetracted"]:
            self.arm.setPosition("FullyRetracted")
        elif switches["Optimized"]:
            self.arm.setPosition("Optimized")
        elif switches["Substation"]:
            self.arm.setPosition("Substation")
        elif switches["Floor"]:
            self.arm.setPosition("Floor")
        elif switches["BottomSlot"]:
            self.arm.setPosition("BottomSlot")
        elif switches["MidCone"]:
            self.arm.setPosition("MidConePrep")
        elif switches["MidCube"]:
            self.arm.setPosition("MidCube")
        elif switches["HighCone"]:
            self.arm.setPosition("HighConePrep")
        elif switches["HighCube"]:
            self.arm.setPosition("HighCube")
    
    def getSelectedGridSlot(self):
        grid, position = self.selectedGridSlot
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed: # are we on the red alliance, if so we have to flip the grid somewhat
            grid = 4 - grid
            if position > 6: # low grid
                if position == 9:
                    position = 7
                elif position == 7:
                    position = 9
            elif position < 4: # high grid
                if position == 3:
                    position = 1
                elif position == 1:
                    position = 3
            else: # mid grid
                if position == 6:
                    position = 4
                elif position == 4:
                    position = 6
        return (grid - 1, position - 1)