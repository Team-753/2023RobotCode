import wpilib
import ctre
from ctre.sensors import AbsoluteSensorRange
import commands2

class ArmSubSystem(commands2.SubsystemBase):
    ''''''
    
    constants = { # what encoder value the arm's lead screw motor should go to according to the needed position
        "fullyRetracted": 0.0,
        "substation": 37.9,
        "floor": 42,
        "highConePrep": 35.45,
        "highConePlacement": 37.16,
        "midConePrep": 37.6,
        "midConePlacement": 38.5,
        "highCube": 0.0,
        "midCube": 0.0,
        "optimized": 21.5
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
        self.armFalcon.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 250)
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
        self.armFalcon.configAllSettings(armFalconConfig, 250)
        wpilib.SmartDashboard.putBoolean("arm zeroed", False)

    def periodic(self) -> None:
        ''' Runs every 20ms in all modes, keep that in mind Joe. '''
        '''
        TODO:
        - Implement Position PID controller on whatever motor we end up going with, if falcon 500 internal PID is feasible though we may
        need to switch over PID's based on how far out the arm is extended.
        - Returns: whether it is within the setpoint threshold, very important for auto stuff
        '''
        wpilib.SmartDashboard.putBoolean("arm limit switch", self.limitSwitch.get())
        if not self.zeroed: # has the arm set its setpoint yet?
            if self.limitSwitch.get(): # is the switch pressed
                self.armFalcon.set(ctre.TalonFXControlMode.PercentOutput, 0) # stopping the motor
                self.armFalcon.setNeutralMode(ctre.NeutralMode.Coast) # coasting the motor
                self.armFalcon.setSelectedSensorPosition(0.0, 0, 250) # zeroing the motor
                self.zeroed = True
                wpilib.SmartDashboard.putBoolean("arm zeroed", True)
            else: # the switch isn't pressed yet; we haven't reached the bottom
                self.armFalcon.set(ctre.TalonFXControlMode.PercentOutput, -0.2) # assuming negative makes it go down
        else: # arm is zeroed, proceed as normal
            self.armFalcon.set(ctre.ControlMode.Position, self.targetValue / self.encoderTicksToDistanceConversionFactor)
            wpilib.SmartDashboard.putNumber("arm target position", self.targetValue)
            wpilib.SmartDashboard.putNumber("arm actual position", self.armFalcon.getSelectedSensorPosition(0) * self.encoderTicksToDistanceConversionFactor)
        return super().periodic()
    
    def reZero(self):
        ''' In a perfect world this should never have to be used but I'm putting it here just in case. '''
        self.zeroed = False

    def setPosition(self, position: str):
        self.targetValue = self.constants[position] # turning a string position into a value via a dictionary
        wpilib.SmartDashboard.putNumber("arm target position", self.targetValue)
        
    def coast(self):
        ''''''
        
    def brake(self):
        ''''''
        
    def manualControlIncrementor(self, scalar: float):
        ''' As the function name says, will need some kind of linear/nonlinear scalar value as the arm speed is very non linear. '''
        increment = scalar * self.maxSpeedPer30MS
        if (self.targetValue + increment > 0 and self.targetValue + increment < self.maxHeightInches):
            self.targetValue += increment
        # DEBUG: print(self.targetValue)