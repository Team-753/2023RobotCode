from pathplannerlib import PathPlannerTrajectory

from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem

from commands.ppSwerveControllerCommand import PPSwerveDriveController
from commands.followPathWithEvents import FollowPathWithEvents
from wpilib import DriverStation

from wpimath import controller, geometry
import commands2
from commands2 import cmd
from typing import List

class SwerveAutoBuilder:
    
    def __init__(self, poseEstimator: PoseEstimatorSubsystem, 
                 driveTrain: DriveTrainSubSystem,
                 eventMap: dict, 
                 useAllianceColor: bool, 
                 translationConstants: dict, 
                 rotationConstants: List,  
                 tolerance: geometry.Pose2d) -> None:
        self.eventMap = eventMap
        self.useAllianceColor = useAllianceColor
        self.poseEstimator = poseEstimator
        self.driveTrain = driveTrain
        self.translationConstants = translationConstants
        self.rotationConstants = rotationConstants
        self.tolerance = tolerance
        
    def followPath(self, trajectory: PathPlannerTrajectory):
        return PPSwerveDriveController(trajectory, 
                                       self.driveTrain, 
                                       self.poseEstimator, 
                                       self.PIDControllerFromConstants(self.translationConstants), 
                                       self.PIDControllerFromConstants(self.translationConstants),
                                       self.rotationConstants,
                                       self.useAllianceColor,
                                       self.tolerance
                                       )
    
    def followPathGroup(self, pathGroup: List[PathPlannerTrajectory]):
        commands = []
        for trajectory in pathGroup:
            commands.append(self.followPath(trajectory))
        return cmd.sequence(commands)
    
    def followPathWithEvents(self, trajectory: PathPlannerTrajectory):
        return FollowPathWithEvents(self.followPath(trajectory), trajectory.getMarkers(), self.eventMap)
    
    def followPathGroupWithEvents(self, pathGroup: List[PathPlannerTrajectory]):
        commands = []
        for trajectory in pathGroup:
            commands.append(self.followPathWithEvents(trajectory))
        return cmd.sequence(commands)
    
    def resetPose(self, trajectory: PathPlannerTrajectory):
        return cmd.runOnce(lambda: self.__rPCmd(trajectory), [self.poseEstimator])
    
    def __rPCmd(self, trajectory: PathPlannerTrajectory):
        initialState = trajectory.getInitialState()
        if (self.useAllianceColor):
            initialState = PathPlannerTrajectory.transformStateForAlliance(initialState, DriverStation.getAlliance())
        self.poseEstimator.setCurrentPose(geometry.Pose2d(initialState.pose.translation(), initialState.holonomicRotation))
        
    def wrappedEventCommand(self, eventCommand: commands2.Command) -> commands2.CommandBase:
        return commands2.FunctionalCommand(
            eventCommand.initialize(),
            eventCommand.execute(),
            eventCommand.end(),
            eventCommand.isFinished(),
            eventCommand.getRequirements()
        )
    
    def getStopEventCommands(self, stopEvent: PathPlannerTrajectory.StopEvent) -> commands2.CommandBase:
        commands = []
        
        startIndex = 1 if stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE else 0
        while startIndex < len(stopEvent.names):
            name = stopEvent.names[startIndex]
            if (self.eventMap[name] != None):
                commands.append(self.wrappedEventCommand(self.eventMap[name]))
        if stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.SEQUENTIAL:
            return cmd.sequence(commands)
        elif stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL:
            return cmd.parallel(commands)
        elif stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE:
            deadline = self.wrappedEventCommand(self.eventMap[stopEvent.names[0]]) if stopEvent.names[0] in self.eventMap else cmd.none()
            return cmd.deadline(deadline, commands)
        else:
            raise Exception("Invalid stop event execution behavior: {}".format(stopEvent.executionBehavior))
        
    def stopEventGroup(self, stopEvent: PathPlannerTrajectory.StopEvent):
        if len(stopEvent.names) == 0:
            return cmd.wait(stopEvent.waitTime)
        
        eventCommands = self.getStopEventCommands(stopEvent)

        if stopEvent.waitBehavior == PathPlannerTrajectory.StopEvent.WaitBehavior.BEFORE:
            return cmd.sequence(cmd.waitSeconds(stopEvent.waitTime), eventCommands)
        elif stopEvent.waitBehavior == PathPlannerTrajectory.StopEvent.WaitBehavior.AFTER:
            return cmd.sequence(eventCommands, cmd.waitSeconds(stopEvent.waitTime))
        elif stopEvent.waitBehavior == PathPlannerTrajectory.StopEvent.WaitBehavior.DEADLINE:
            return cmd.deadline(cmd.waitSeconds(stopEvent.waitTime), eventCommands)
        elif stopEvent.waitBehavior == PathPlannerTrajectory.StopEvent.WaitBehavior.MINIMUM:
            return cmd.parallel(cmd.waitSeconds(stopEvent.waitTime), eventCommands)
        else:
            return eventCommands

    def fullAuto(self, trajectory: PathPlannerTrajectory) -> commands2.CommandBase:
        return self.fullAuto([trajectory])
    
    def fullAuto(self, trajectoryGroup: List[PathPlannerTrajectory]) -> commands2.CommandBase:
        commands = []
        
        commands.append(self.resetPose(trajectoryGroup[0]))
        
        for trajectory in trajectoryGroup:
            commands.append(self.stopEventGroup(trajectory.getStartStopEvent()))
            commands.append(self.followPathWithEvents(trajectory))
        
        commands.append(self.stopEventGroup(trajectoryGroup[len(trajectoryGroup) - 1].getEndStopEvent()))
        
        return cmd.sequence(commands)
        
    def PIDControllerFromConstants(self, constants: dict):
        ''' Creates and returns a PID controller based off the dictionary (hashmap) parameter'''
        return controller.PIDController(constants["kP"], constants["kI"], constants["kD"], constants["period"])