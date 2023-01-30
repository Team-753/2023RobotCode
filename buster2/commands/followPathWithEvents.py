from pathplannerlib import PathPlannerTrajectory
from wpilib import Timer
import commands2
from typing import List

class FollowPathWithEvents(commands2.CommandBase):
    currentCommands = {}
    unpassedMarkers = []
    timer = Timer()
    finished = True
    
    def __init__(self, pathFollowingCommand: commands2.Command, pathMarkers: List[PathPlannerTrajectory.EventMarker], eventMap: dict) -> None:
        super().__init__()
        self.markers = pathMarkers
        self.pathFollowingCommand = pathFollowingCommand
        self.eventMap = eventMap
        reqs = []
        for marker in self.markers:
            reqs.append(self.eventMap.get(marker).getRequirements())
        #reqs.append(self.pathFollowingCommand.getRequirements())
        self.addRequirements(reqs)
        
    def initialize(self) -> None:
        self.finished = False
        
        self.currentCommands.clear()
        
        self.unpassedMarkers.clear()
        self.unpassedMarkers = self.markers.copy()
        
        self.timer.reset()
        self.timer.start()
        
        self.pathFollowingCommand.initialize()
        self.currentCommands[self.pathFollowingCommand] = True
        
    def execute(self) -> None:
        for runningCommand in self.currentCommands:
            if (not self.currentCommands.get(runningCommand)):
                continue
            
            runningCommand.execute()
            
            if (runningCommand.isFinished()):
                runningCommand.end(False)
                self.currentCommands[runningCommand] = False
                if (runningCommand == self.pathFollowingCommand):
                    self.finished = True
        
        currentTime = self.timer.get()
        if (len(self.unpassedMarkers) > 0 and currentTime >= self.unpassedMarkers[0].time):
            marker = self.unpassedMarkers.pop(0)
            
            for name in marker.names:
                if (self.eventMap.get(name) != None):
                    eventCommand = self.eventMap.get(name)
                    
                    for runningCommand in self.currentCommands:
                        if (not self.currentCommands.get(runningCommand)):
                            continue
                        
                        eventCommand.initialize()
                        self.currentCommands[eventCommand] = True # possible issues with replacement of current command based on key duplicates???
    
    def end(self, interrupted: bool) -> None:
        for runningCommand in self.currentCommands:
            if (self.currentCommands.get(runningCommand)):
                runningCommand.end(True)
    
    def isFinished(self) -> bool:
        return self.finished