import wpilib
import commands2
from robotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.robotContainer = RobotContainer(self)
        self.autoCommand = commands2.Command()
        
    def disabledInit(self) -> None:
        ''''''
        return super().disabledInit()
            
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        return super().testPeriodic()
    
    def autonomousInit(self):
        self.autoCommand = self.robotContainer.getAutonomousCommand()
        if (self.autoCommand != commands2.Command()):
            self.autoCommand.schedule()
        
    def autonomousPeriodic(self):
        pass
        
    def teleopInit(self):
        if (self.autoCommand != commands2.Command()):
            self.autoCommand.cancel()
        self.robotContainer.teleopInit()
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        pass
    
    def disabledPeriodic(self):
        ''' Runs while the robot is idle '''
    
    def disabledInit(self) -> None:
        '''self.driveTrain.coast()'''
        
    

if __name__ == "__main__":
    wpilib.run(MyRobot)