from wpimath import geometry
from subsystems.arm import Arm
from subsystems.mandible import Mandible
from math import pi

class CommandController:
    ''' This exists because I didn't want to take the time to learn wpilib commands, also I hate myself so yeah...
    Anyway, this is basically what acts on those fancy macro buttons we have this year to automate large parts of the game.'''
    placementConstants = { # the positioning constants associated with each and every command 
        "leftGrid": {
            "topLeftCone": ["highCone", "cubeMode", geometry.Pose2d(geometry.Translation2d(), geometry.Rotation2d(pi))] # convention is as follows: arm key, mandible key, chassis location
        },
        "midGrid": {
            
        },
        "rightGrid": {
            
        }
    }
    
    macros = { # for things either the driver or auto may want to do at some point, not a full automation, just doing something...
        
    }
    def __init__(self) -> None:
        pass
    
    