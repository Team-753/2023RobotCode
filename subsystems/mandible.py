import wpilib
from wpilib import DoubleSolenoid, PneumaticsModuleType
from ctre import VictorSPX, VictorSPXControlMode, NeutralMode
import playingwithfusion
import commands2

class MandibleSubSystem(commands2.SubsystemBase):
    ''''''
    state = "cube"
    gamePieceInPossessionDistance = 80 # in millimeters from the distance sensor, make this bigger later to be safe
    
    def __init__(self, config: dict) -> None:
        super().__init__()
        self.config = config
        self.actuator = DoubleSolenoid(self.config["RobotDefaultSettings"]["PCM_ID"], PneumaticsModuleType.REVPH, self.config["MandibleConfig"]["DoubleSolenoid"]["ForwardChannel"], self.config["MandibleConfig"]["DoubleSolenoid"]["ReverseChannel"])
        self.compressor = wpilib.Compressor(self.config["RobotDefaultSettings"]["PCM_ID"], PneumaticsModuleType.REVPH)
        self.compressor.enableDigital()
        self.leftMotor = VictorSPX(self.config["MandibleConfig"]["LeftMotorID"])
        self.leftMotor.setNeutralMode(NeutralMode.Brake)
        self.rightMotor = VictorSPX(self.config["MandibleConfig"]["RightMotorID"])
        self.rightMotor.setNeutralMode(NeutralMode.Brake)
        self.rightMotor.setInverted(True)
        self.distanceSensor = playingwithfusion.TimeOfFlight(self.config["MandibleConfig"]["DistanceSensorID"])
        self.distanceSensor.setRangingMode(self.distanceSensor.RangingMode.kShort, 30)
        #self.distanceSensor.setRangeOfInterest(p1, p1, p3, p4) NOTE: may need this
        # TODO: Invert one of these motors once we get the mandible together
    
    def doSomething(self, todo: str):
        pass
    
    def contract(self):
        ''' Pretty self-explanatory '''
        self.state = "cone"
        self.actuator.set(self.actuator.Value.kForward)
        
    def release(self):
        ''' Pretty self-explanatory '''
        self.state = "cube"
        self.actuator.set(self.actuator.Value.kReverse)
        
    def intake(self) -> bool:
        ''' NOTE: Needs to check whether or not game piece is fully in the mandible, distance sensor??? will return false until true. '''
        if self.inControlOfPiece():
            self.leftMotor.set(VictorSPXControlMode.PercentOutput, -0.25)
            self.rightMotor.set(VictorSPXControlMode.PercentOutput, -0.25)
            return True
        else:
            self.leftMotor.set(VictorSPXControlMode.PercentOutput, -0.5)
            self.rightMotor.set(VictorSPXControlMode.PercentOutput, -0.5)
            return False
        
    def outtake(self):
        ''' Pretty self-explantory '''
        self.leftMotor.set(VictorSPXControlMode.PercentOutput, 0.25)
        self.rightMotor.set(VictorSPXControlMode.PercentOutput, 0.25)
        
    def stop(self):
        ''''''
        self.leftMotor.set(VictorSPXControlMode.PercentOutput, 0)
        self.rightMotor.set(VictorSPXControlMode.PercentOutput, 0)
    
    def getState(self):
        return self.state
    
    def setState(self, stateToSet: str):
        ''' Sets the desired mandible state, if the state is already set: do nothing.
        stateToSet: 'cone', 'cube' '''
        if stateToSet == "cone" and stateToSet != self.state: # we want a cone
            self.contract()
            #print("Mandible Contracting")
        elif stateToSet == "cube" and stateToSet != self.state: # we want a cube
            self.release()
            #print("Mandible Releasing")
        
    def inControlOfPiece(self) -> bool:
        '''
        psuedocode:
        import distance sensor class
        if (sensor.inRange()):
            return True
        else:
            return False
        '''
        distance = self.distanceSensor.getRange()
        wpilib.SmartDashboard.putNumber("distance sensor", distance)
        if distance <= self.gamePieceInPossessionDistance:
            return True
        else:
            return False
    
    def isGamePieceInFront(self) -> bool:
        ''' Need to test the distance sensor range first, but this would allow us to bypass operator confirmation of a game piece
        being place in front of the robot on the substation. '''
        
    def cubePeriodic(self) -> None:
        ''' Ensures we hold onto a cube once it is in our possession '''
        '''if self.inControlOfPiece() and self.state == "cube":
            self.intake()
        else:
            self.stop()'''
    