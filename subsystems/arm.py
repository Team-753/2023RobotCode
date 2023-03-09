import wpilib
import ctre
from ctre.sensors import AbsoluteSensorRange
import commands2

class ArmSubSystem(commands2.SubsystemBase):
    ''''''
    
    constants = { # what encoder value the arm's lead screw motor should go to according to the needed position
        "FullyRetracted": 0.0,
        "Substation": 37.9,
        "Floor": 41.75,
        "BottomPlacement": 40.4,
        "HighConePrep": 35.2,
        "HighConePlacement": 37.16,
        "MidConePrep": 37.2, # 37.6
        "MidConePlacement": 38.5,
        "HighCube": 37.1,
        "MidCube": 38.54,
        "Optimized": 21.5
    }
    targetValue = 0
    maxHeightInches = 42.5
    encoderTicksToDistanceConversionFactor = 0.5 / (4 * 2048) # conversion factor, see details below
    maxSpeedPer30MS = 12 * 0.03 # 12 inches per second
    '''
    1/2 inches of travel per lead screw rotation
    12:1 gear ratio through planetary
    2048 counts per rotation
    so for example: 1 inch of travel is 49,152 encoder ticks of rotation
    '''
    zeroed = False # has the arm been zeroed using the limit switch yet
    
    def __init__(self, config: dict) -> None:
        super().__init__()
        self.config = config
        self.limitSwitch = wpilib.DigitalInput(self.config["ArmConfig"]["LimitSwitchID"])
        self.armFalcon = ctre.TalonFX(self.config["ArmConfig"]["FalconID"])
        self.armFalcon.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 50)
        armFalconConfig = ctre.TalonFXConfiguration()
        armFalconConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
        pidConfig = self.config["ArmConfig"]["FalconPIDValues"]
        armFalconConfig.slot0.kP = pidConfig["kP"]
        armFalconConfig.slot0.kI = pidConfig["kI"]
        armFalconConfig.slot0.kD = pidConfig["kD"]
        armFalconConfig.slot0.kF = pidConfig["kF"]
        armFalconConfig.slot0.integralZone = pidConfig["iZone"]
        armFalconConfig.slot0.allowableClosedloopError = pidConfig["error"]
        currentConfig = ctre.SupplyCurrentLimitConfiguration()
        currentConfig.currentLimit = pidConfig["currentLimit"]
        armFalconConfig.supplyCurrLimit = currentConfig
        self.armFalcon.setNeutralMode(ctre.NeutralMode.Brake)
        self.armFalcon.configAllSettings(armFalconConfig, 50)
        self.armStringPosition = "fullyRetracted"
        self.tolerance = self.config["ArmConfig"]["autoPlacementTolerance"]

    def periodic(self) -> None:
        ''' Runs every 20ms in all modes, keep that in mind Joe. '''
        '''
        TODO:
        - Implement Position PID controller on whatever motor we end up going with, if falcon 500 internal PID is feasible though we may
        need to switch over PID's based on how far out the arm is extended.
        - Returns: whether it is within the setpoint threshold, very important for auto stuff
        '''
        wpilib.SmartDashboard.putBoolean("Arm Limit Switch", self.limitSwitch.get())
        if not self.zeroed: # has the arm set its setpoint yet?
            if self.limitSwitch.get(): # is the switch pressed
                self.armFalcon.set(ctre.TalonFXControlMode.PercentOutput, 0) # stopping the motor
                self.armFalcon.setNeutralMode(ctre.NeutralMode.Coast) # coasting the motor
                self.armFalcon.setSelectedSensorPosition(0.0, 0, 50) # zeroing the motor
                self.zeroed = True
            else: # the switch isn't pressed yet; we haven't reached the bottom
                self.armFalcon.set(ctre.TalonFXControlMode.PercentOutput, -0.2) # assuming negative makes it go down
        else: # arm is zeroed, proceed as normal
            self.armFalcon.set(ctre.ControlMode.Position, self.targetValue / self.encoderTicksToDistanceConversionFactor)
            wpilib.SmartDashboard.putNumber("Arm Position", self.armFalcon.getSelectedSensorPosition(0) * self.encoderTicksToDistanceConversionFactor)
        return super().periodic()
    
    def reZero(self):
        ''' In a perfect world this should never have to be used but I'm putting it here just in case. '''
        self.zeroed = False

    def setPosition(self, position: str):
        self.targetValue = self.constants[position] # turning a string position into a value via a dictionary
        self.armStringPosition = position
        wpilib.SmartDashboard.putString("Target Arm Position", position)

    def atSetpoint(self):
        if abs(self.targetValue - self.armFalcon.getSelectedSensorPosition(0) * self.encoderTicksToDistanceConversionFactor) < self.tolerance: # the arm is within the tolerance position threshold
            return True
        return False
            
    
    def manualControlIncrementor(self, scalar: float):
        ''' As the function name says, will need some kind of linear/nonlinear scalar value as the arm speed is very non linear. '''
        increment = scalar * self.maxSpeedPer30MS
        if (self.targetValue + increment > 0 and self.targetValue + increment < self.maxHeightInches):
            self.targetValue += increment
        # DEBUG: print(self.targetValue)