import commands2
from auto.swerveAutoBuilder import SwerveAutoBuilder
from subsystems.mandible import MandibleSubSystem
from subsystems.arm import ArmSubSystem
from subsystems.poseEstimator import PoseEstimatorSubsystem

class SubstationPickupCommand(commands2.CommandBase):
    '''
    Command for near-autonomously picking up game pieces from the substation
    General chain of events:
    - Button pressed by one of the drivers, activating the macro. The driver with the joystick should go handsfree at this point.
    - command starts
    - Robot starts moving arm into position right away, if it is not already in position
    - Robot moves into and holds a position to where the arm is close to yet grabbing the game piece (whatever game piece it may be)
    - Command waits for auxiliary/driver input confirmation that the game piece is in position and lined up over the camera.
    - Once confirmation input is received, robot drives forward to the exact right point, grabbing the game piece, the robot will stop intaking the mandible once it detects a game piece, as per the mandible intake command
    - Controls are released back to the driver, TODO for this: Add a feature where once the game piece is picked up, the robot will automatically start following a trajectory back towards the grids
    '''
    
    def __init__(self, SwerveAutoBuilder: SwerveAutoBuilder, Mandible: MandibleSubSystem, Arm: ArmSubSystem, PoseEstimator: PoseEstimatorSubsystem) -> None:
        super().__init__()
        self.swerveAutoBuilder = SwerveAutoBuilder
        self.mandible = Mandible
        self.arm = Arm
        self.poseEstimator = PoseEstimator
        self.blueAllianceAprilTagPose = self.poseEstimator.getTagPose(4) # if we are the red alliance the swerveAutoBuilder should automajically switch everything around
        
        
        