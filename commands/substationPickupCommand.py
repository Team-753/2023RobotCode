import commands2
from commands2 import cmd, button
from wpimath import geometry, controller, kinematics
import math
import wpilib

from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem
from networktables import NetworkTable

import pathplannerlib

class SubstationPickupCommand(commands2.CommandBase):
    '''
    Command for near-autonomously picking up game pieces from the substation.
    \nHere is the general chain of events:
    - Button pressed by one of the drivers, activating the macro. The driver with the joystick should go handsfree at this point.
    - command starts
    - Robot starts moving arm into position right away, if it is not already in position
    - Robot moves into and holds a position to where the arm is close to yet grabbing the game piece (whatever game piece it may be)
    - Command waits for auxiliary/driver input confirmation that the game piece is in position and lined up over the camera.
    - Once confirmation input is received, robot drives forward to the exact right point, grabbing the game piece, the robot will stop intaking the mandible once it detects a game piece, as per the mandible intake command
    - Controls are released back to the driver, TODO for this: Add a feature where once the game piece is picked up, the robot will automatically start following a trajectory back towards the grids
    '''
    
    # TEST VAR INITS, in degrees and meters respectively
    robotTargetVerticalX = 0 # measure on practice field
    grabTargetVerticalX = 0 # measure
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, DriveTrain: DriveTrainSubSystem, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Constraints: pathplannerlib.PathConstraints, DriverCommandJoystick: button.CommandJoystick, LLTable: NetworkTable, PIDCONSTANTS: dict) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm])
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.driveTrain = DriveTrain
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.driverCommandJoystick = DriverCommandJoystick
        self.constraints = Constraints
        self.LLTable = LLTable
        self.PIDCONSTANTS = PIDCONSTANTS
    
    def initialize(self) -> None:
        self.finished = False
        self.allianceFactor = -1
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.allianceFactor = 1
        self.xController = controller.PIDController(self.PIDCONSTANTS["translationPIDConstants"]["kP"], self.PIDCONSTANTS["translationPIDConstants"]["kI"], self.PIDCONSTANTS["translationPIDConstants"]["kD"], self.PIDCONSTANTS["translationPIDConstants"]["period"])
        self.yController = controller.PIDController(self.PIDCONSTANTS["translationPIDConstants"]["kP"], self.PIDCONSTANTS["translationPIDConstants"]["kI"], self.PIDCONSTANTS["translationPIDConstants"]["kD"], self.PIDCONSTANTS["translationPIDConstants"]["period"])
        self.zController = controller.PIDController(self.PIDCONSTANTS["rotationPIDConstants"]["kP"], self.PIDCONSTANTS["rotationPIDConstants"]["kI"], self.PIDCONSTANTS["rotationPIDConstants"]["kD"], self.PIDCONSTANTS["rotationPIDConstants"]["period"])
        self.angleOffset = 0
    
    def execute(self) -> None:
        currentPose = self.poseEstimator.getCurrentPose()
        if self.driverCommandJoystick.getRawButton(1):
            targetPose = geometry.Pose2d(geometry.Translation2d(self.grabTargetVerticalX, currentPose.Y()), geometry.Rotation2d(0.5 * math.pi + (self.allianceFactor * 0.5 * math.pi)))
            onLeFlyTrajectory = self.swerveAutoBuilder.followPath(pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(currentPose, self.driveTrain.actualChassisSpeeds()), pathplannerlib.PathPoint.fromCurrentHolonomicState(targetPose, kinematics.ChassisSpeeds(0, 0, 0))]))
        else:
            if self.LLTable.getEntry('tv') == 1:
                if self.LLTable.getEntry('tclass') == 'cone':
                    if self.mandible.state != 'Cone':
                        self.mandible.setState('Cone')
                else:
                    if self.mandible.state != 'Cube':
                        self.mandible.setState('Cube')
                self.angleOffset = self.LLTable.getEntry('tx')
                wpilib.SmartDashboard.putNumber("LL Angle TX", self.angleOffset)
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool: # driver should just let go of the button, no need for this
        return super().isFinished() # TODO: Possibly have robot start driving itself towards the middle of the field after it detects a successful game piece pickup