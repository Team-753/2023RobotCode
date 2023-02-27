import commands2
from commands2 import cmd, button
from wpimath import geometry

from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem
from subsystems.driveTrain import DriveTrainSubSystem

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
    xIOffset = -2 # field relative, how far the initial pose is from the substation apriltag
    yIOffset = 1 # field relative, how far down or up the initial pose is from the substation apriltag
    
    xFOffset = -1.5 # field relative, how far the final pose is from the substation apriltag
    yFOffset = 1 # field relative, how far down or up the final pose is from the substation apriltag
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem, Constraints: pathplannerlib.PathConstraints, EventMap: dict, DriverCommandJoystick: button.CommandJoystick) -> None:
        super().__init__()
        self.addRequirements([Mandible, Arm])
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.eventMap = EventMap
        self.driverCommandJoystick = DriverCommandJoystick
        self.constraints = Constraints
    
    def initialize(self) -> None:
        self.currentPose = self.poseEstimator.getCurrentPose()
        self.blueAllianceAprilTagPose = self.poseEstimator.getTagPose(4).toPose2d() # if we are the red alliance the swerveAutoBuilder should automajically switch everything around
        # first step is to figure out which side of the substation we are closer to, we can just do this using the Y-component to avoid alliance conflict
        if (self.currentPose.Y() < self.blueAllianceAprilTagPose.Y()): # we are closer to the bottom side of the blue alliance substation, flip our y offset
            self.yIOffset *= -1
            self.yFOffset *= -1
        self.initialTargetPose = geometry.Pose2d(geometry.Translation2d(x=self.xIOffset, y=self.yIOffset), geometry.Rotation2d(0)).transformBy(geometry.Transform2d(self.blueAllianceAprilTagPose.translation(), geometry.Rotation2d())) # finding our new offset initial pose
        self.finalTargetPose = geometry.Pose2d(geometry.Translation2d(x=self.xFOffset, y=self.yFOffset), geometry.Rotation2d(0)).transformBy(geometry.Transform2d(self.blueAllianceAprilTagPose.translation(), geometry.Rotation2d())) # finding our new offset final pose
        self.currentPose = self.poseEstimator.getCurrentPose()
        initialPath = pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint.fromCurrentHolonomicState(self.currentPose, self.poseEstimator.chassisSpeeds), pathplannerlib.PathPoint(self.initialTargetPose.translation(), geometry.Rotation2d(), self.initialTargetPose.rotation())])
        self.followInitialPathCommand = self.swerveAutoBuilder.followPath(initialPath)
        finalPath = pathplannerlib.PathPlanner.generatePath(self.constraints, [pathplannerlib.PathPoint(self.initialTargetPose.translation(), geometry.Rotation2d(), self.initialTargetPose.rotation()), pathplannerlib.PathPoint(self.finalTargetPose.translation(), geometry.Rotation2d(), self.finalTargetPose.rotation())])
        self.followFinalPathCommand = self.swerveAutoBuilder.followPath(finalPath)
        self.secondStageCommand = commands2.ParallelCommandGroup([self.followFinalPathCommand, self.eventMap["IntakeMandible"]])
        self.confirmationButton = button.JoystickButton(self.driverCommandJoystick, 12) # find the correct number later on
        self.confirmationButton.whenPressed(self.secondStageCommand)
        cmd.parallel([self.followInitialPathCommand, self.eventMap["armSubstation"]])
    
    def execute(self) -> None:
        pass
    
    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.mandible.coast()
            self.arm.brake()
        return super().end(interrupted)
    
    def isFinished(self) -> bool: # driver should just let go of the button, no need for this
        return super().isFinished() # TODO: Possibly have robot start driving itself towards the middle of the field after it detects a successful game piece pickup