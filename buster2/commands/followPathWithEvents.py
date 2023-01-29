from pathplannerlib import PathPlannerTrajectory
from wpilib import Timer
import commands2

class FollowPathWithEvents(commands2.CommandBase):
    currentCommands = {}
    unpassedMarkers = []
    
    def __init__(self) -> None:
        super().__init__()