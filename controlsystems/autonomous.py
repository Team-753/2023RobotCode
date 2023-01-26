import wpilib
from pathplannerlib import PathPlanner, PathPlannerTrajectory, PathConstraints, PathPoint

class Autonomous:
    def __init__(self, config: dict, mode) -> None:
        self.config = config
        self.alliance = wpilib.DriverStation.getAlliance()
        self.mode = mode
        
        self.maxVel = self.config["autonomousSettings"]["autoVelLimit"]
        self.maxAcc = self.config["autonomousSettings"]["autoAccLimit"]
        '''
        options for modes:
        goto
        autopath
        '''
        
    def goTo(self, x, y, rotation):
        pass
    
    def followAprilTag(self):
        ''' Stays one meter in front of the corresponding apriltag '''
        trajectory = PathPlanner.generatePath()
    
    def loadPath(self, pathname: str):
        ''''''
        #path = PathPlanner.loadPath(pathname, self.maxVel, self.maxAcc, False)
    
    def periodic(self):
        if (self.mode == "autopath"):
            pass