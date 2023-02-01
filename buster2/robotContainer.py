import commands2
from commands2 import button, cmd
import wpilib
import os
import json
from wpimath import geometry
from math import radians
import pathplannerlib
from typing import List

from subsystems.driveTrain import DriveTrainSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem

from commands.mandibleIntakeCommand import MandibleIntakeCommand

from auto.swerveAutoBuilder import SwerveAutoBuilder

class RobotContainer:
    PLACEHOLDER = geometry.Pose2d()
    
    folderPath = os.path.dirname(os.path.abspath(__file__))
    filePath = os.path.join(folderPath, 'config.json')
    with open (filePath, "r") as f1:
        config = json.load(f1)
        
    selectedAutoName = "Drive Forward" # NOTE: Change this to change active auto
    
    def __init__(self, MyRobot: commands2.TimedCommandRobot) -> None:
        # subsystems
        self.driveTrain = DriveTrainSubSystem(MyRobot, self.config)
        self.poseEstimator = PoseEstimatorSubsystem(MyRobot, self.driveTrain, self.PLACEHOLDER, self.config)
        self.mandible = MandibleSubSystem(98, 99)
        self.arm = ArmSubSystem()
        
        # commands
        self.MandibleIntakeCommand = MandibleIntakeCommand(self.mandible)
        
        # buttons
        self.joystick = button.CommandJoystick(0)
        
        #subsystem configuration
        self.driveTrain.setDefaultCommand(cmd.run(lambda: self.driveTrain.joystickDrive(self.getJoystickInput()), [self.driveTrain]))
        
        # additional configuration
        
        self.pathConstraints = pathplannerlib.PathConstraints(self.config["autonomousSettings"]["autoVelLimit"], self.config["autonomousSettings"]["autoAccLimit"])
        self.configureButtonBindings()
        self.generateSimpleAutonomousCommands()
        self.generateAutonomousMarkers()
        
        self.SwerveAutoBuilder = SwerveAutoBuilder(self.poseEstimator, 
                                                   self.driveTrain, 
                                                   self.eventMap, 
                                                   True, 
                                                   self.config["autonomousSettings"]["translationPIDConstants"],
                                                   self.config["autonomousSettings"]["translationPIDConstants"],
                                                   geometry.Pose2d(geometry.Translation2d(x=self.config["autonomousSettings"]["xPoseToleranceMeters"], 
                                                                                          y=self.config["autonomousSettings"]["yPoseToleranceMeters"]), 
                                                                                          geometry.Rotation2d(radians(self.config["autonomousSettings"]["thetaPoseToleranceDegrees"]
                                                                                                                      )
                                                                                                              )
                                                                                          )
                                                   )
    
    def configureButtonBindings(self):
        self.joystickButtonTwo = button.JoystickButton(self.joystick, 2)
        self.joystickButtonTwo.whileHeld(cmd.run(lambda: self.driveTrain.xMode(), [self.driveTrain]))
        
    def getJoystickInput(self):
        inputs = (self.joystick.getX(), self.joystick.getY(), self.joystick.getZ())
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = self.config["driverStation"]["joystickDeadZones"][(list(self.config["driverStation"]["joystickDeadZones"])[idx])]
            if abs(input) > threshold: 
                adjustedValue = (abs(input) - threshold) / (1 - threshold)
                if input < 0 and adjustedValue != 0:
                    adjustedValue = -adjustedValue
            else:
                adjustedValue = 0
            adjustedInputs.append(adjustedValue)
        return adjustedInputs[0], adjustedInputs[1], adjustedInputs[2]
    
    def evaluateDeadzones(self, inputs):
        '''This method takes in a list consisting of x input, y input, z input
        The magnitude of the units has to be less than 1.
        Returns the list of inputs with zero in place of values less than their respective deadzones.'''
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = self.config["driverStation"]["joystickDeadZones"][(list(self.config["driverStation"]["joystickDeadZones"])[idx])]
            if abs(input) > threshold: 
                adjustedValue = (abs(input) - threshold) / (1 - threshold)
                if input < 0 and adjustedValue != 0:
                    adjustedValue = -adjustedValue
            else:
                adjustedValue = 0
            adjustedInputs.append(adjustedValue)
        return adjustedInputs
    
    def getAutonomousCommand(self):
        return self.SwerveAutoBuilder.fullAuto(self.getPathGroup(self.selectedAutoName))
    
    def getPath(self, pathName: str) -> pathplannerlib.PathPlannerTrajectory:
        return pathplannerlib.PathPlanner.loadPath(pathName, self.pathConstraints, False)
    
    def getPathGroup(self, groupName: str) -> List[pathplannerlib.PathPlannerTrajectory]:
        return pathplannerlib.PathPlanner.loadPathGroup(groupName, [self.pathConstraints], False)
    
    def teleopInit(self):
        self.driveTrain.navx.reset()
    
    def disabledInit(self):
        self.driveTrain.coast()
    
    def generateAutonomousMarkers(self):
        self.eventMap = {
            "OpenMandible": cmd.runOnce(lambda: self.mandible.setState('cube'), [self.mandible]),
            "CloseMandible": cmd.runOnce(lambda: self.mandible.setState("cone"), [self.mandible]),
            "OuttakeMandible": self.MandibleOuttakeCommand,
            "IntakeMandible": self.MandibleIntakeCommand,
            "armFullyRetracted": cmd.runOnce(lambda: self.arm.setPosition("fullyRetracted"), [self.arm]),
            "armSubstation": cmd.runOnce(lambda: self.arm.setPosition("substation"), [self.arm]),
            "armFloor": cmd.runOnce(lambda: self.arm.setPosition("floor"), [self.arm]),
            "armHighCone": cmd.runOnce(lambda: self.arm.setPosition("highCone"), [self.arm]),
            "armMidCone": cmd.runOnce(lambda: self.arm.setPosition("midCone"), [self.arm]),
            "armHighCube": cmd.runOnce(lambda: self.arm.setPosition("highCube"), [self.arm]),
            "armMidCube": cmd.runOnce(lambda: self.arm.setPosition("midCube"), [self.arm])
        }
    
    def generateSimpleAutonomousCommands(self):
        ''' Generates some of the autonomous commands that use *less* logic for use in the auto event map and for manual controls '''
        self.MandibleOuttakeCommand = commands2.SequentialCommandGroup(
            [
                cmd.runOnce(lambda: self.mandible.outtake(), [self.mandible]), 
                commands2.WaitCommand(0.5), # TODO: Test this 0.5 second figure, maybe make this a config variable in the future?
                cmd.runOnce(lambda: self.mandible.coast(), [self.mandible])
            ]
        )