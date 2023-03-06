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
        button.JoystickButton(self.streamDeck, 13).onTrue(cmd.runOnce(lambda: self.arm.setPosition("fullyRetracted"), []))
        button.JoystickButton(self.streamDeck, 14).onTrue(cmd.runOnce(lambda: self.arm.setPosition("optimized"), []))
        button.JoystickButton(self.streamDeck, 15).onTrue(cmd.runOnce(lambda: self.arm.setPosition("substation"), []))
        button.JoystickButton(self.streamDeck, 16).onTrue(cmd.runOnce(lambda: self.arm.setPosition("floor"), []))
        
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
            "GridThree": self.streamDeck.getRawButtonReleased(12)
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
    
    def getSelectedGridSlot(self):
        grid, position = self.selectedGridSlot
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed: # are we on the red alliance, if so we have to flip the grid somewhat
            grid = 4 - grid
            if position > 6: # high grid
                if position == 9:
                    position = 7
                elif position == 7:
                    position = 9
            elif position < 4: # low row
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