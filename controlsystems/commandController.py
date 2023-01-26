from wpimath import geometry
from subsystems.arm import Arm
from subsystems.mandible import Mandible
from math import pi
from wpilib import Timer

class CommandController:
    ''' This exists because I didn't want to take the time to learn wpilib commands, also I hate myself so yeah...
    Anyway, this is basically what acts on those fancy macro buttons we have this year to automate large parts of the game.'''
    auxilaryTimer = Timer()
    placementConstants = { # the drivetrain positioning constants associated with each and every command 
        "leftGrid": {
            "topLeftCone": [geometry.Pose2d(), "highCone", "cone"]
            #"topLeftCone": ["highCone", "cubeMode", geometry.Pose2d(geometry.Translation2d(), geometry.Rotation2d(pi))] # convention is as follows: arm key, mandible key, chassis location
        },
        "midGrid": {
            
        },
        "rightGrid": {
            
        },
        "leftSubstation": [geometry.Pose2d(), geometry.Pose2d()],
        "rightSubstation": [geometry.Pose2d(), geometry.Pose2d()]
    }
    
    macros = { # for things either the driver or auto may want to do at some point, not a full automation, just doing something...
    }
    def __init__(self) -> None:
        self.arm = Arm()
        self.mandible = Mandible()
    
    def placePieceCommand(self, grid: str, gamePiecePosition, inPosition: bool):
        ''' For placing any game piece, anywhere, on any grid '''
        target = self.placementConstants[grid][gamePiecePosition]
        if inPosition:
            # now we move the arm into position, see arm file for armPosition args
            if (self.arm.goToPosition(target[1])): # checking if the arm is in position 
                if target[2] == "cone":
                    self.mandible.release()
                    return True, None 
                else: # t'is a sphube, outtake it, need to time this
                    if self.auxilaryTimer.hasElapsed(0.25): # 0.25 estimated seconds to outtake cube. NOTE: Maybe change this where we use the distance sensor to see if we have outtaked the cube
                        self.mandible.coast()
                        self.auxilaryTimer.stop()
                        self.auxilaryTimer.reset()
                        return True, None
                    elif self.auxilaryTimer.get() == 0: # checking if we have started outtaking yet
                        self.mandible.outtake()
                        self.auxilaryTimer.start()
                        return False, target[0]
                    else:
                        return False, target[0] # the timer is running, the command is not over but we still want the robot to stay in position
                    
        else:
            return False, target[0]
        # returns: (is command done?), (if not done, target robot position)
    def substationPickupCommand(self, side: str, inPositionOne: bool, gamePieceType: str, gamePieceInPosition: bool):
        '''
        Command for picking up game pieces of any type from the substation
        '''
        target = self.placementConstants[side] # see placement constants for side keys
        self.mandible.setState(gamePieceType) # setting this beforehand to show human player the desired game piece
        if inPositionOne: # is the robot in position?
            if (self.arm.goToPosition("substation")): # is the arm in position
                '''now we need operator confirmation that the game piece is ready to be picked up.
                This boolean comes from the gamePieceInPosition parameter'''
                if gamePieceInPosition: # now we move to the pickup stage
                    if (self.mandible.intake()): # we have the game piece, no need to coast, that functionality is built in
                        return True, True, None
                    else:
                        return True, False, target[1] # we dont have the game piece yet, hold/target position; hope and pray
                
            else: # arm isn't in position yet, hold robot position and wait for the arm
                return False, False, target[0]
        else:
            return False, False, target[0]
        # returns: (first stage done?), (are we done now?), posetarget
    
    def autoPickupCommand(self, armPosition: str, gamePieceType: str):
        ''' Same idea as the substation pickup command but doesn't involve as many checks and doesn't manipulate the drivetrain.
        Shouldn't be interrupted by the drivetrain.
        armPosition: string of desired arm position
        gamePieceType: 'cube', 'cone' '''
        self.mandible.setState(gamePieceType) # setting this beforehand because why not
        if (self.arm.goToPosition(armPosition)): # is the arm in position?
            if (self.mandible.intake()): # trying to intake the game piece / do we have it???
                return True # we have the game piece
            else: # we don't have it yet :(
                return False
        else:
            return False # not done
        # returns: (have we picked up the game piece?)
    
    def armToPositionCommand(self, armPosition: str):
        ''' Moves the arm to the given position, this shouldn't be able to be interrupted by anything but other arm commands '''
        if (self.arm.goToPosition(armPosition)):
            return True
        else:
            return False
        
    def setMandibleStateCommand(self, stateToSet: str):
        ''' Sets the mandible to either open or closed.
        stateToSet: 'cube', 'cone' '''
        self.mandible.setState(stateToSet)
        
    def setMandibleNeutralWheelState(self, stateToSet: str):
        ''' Sets the neutral wheel state for the mandible.
        stateToSet: 'coast', 'brake' '''
        self.mandible.setNeutralWheelState(stateToSet)
    
    def mandibleIntake(self):
        ''' Spins the wheels inwards until the game piece is detected '''
        if self.mandible.intake():
            return True
        else:
            return False
        
    def mandibleOuttake(self):
        ''' Spins the wheels outwards '''
        self.mandible.outtake()