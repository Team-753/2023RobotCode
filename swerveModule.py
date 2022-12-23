import ctre
import wpimath.controller
import math
import wpilib

class swerveModule:
    def __init__(self, driveID: int, turnID: int, absoluteID: int, absoluteOffset: float, moduleName: str):
        if moduleName == "frontLeft":
            # do something
            kPTurn, kITurn, kDTurn = 0.007, 0.004, 0.00015
        elif moduleName == "frontRight":
            # do something
            kPTurn, kITurn, kDTurn = 0.007, 0.004, 0.00015
        elif moduleName == "rearLeft":
            # do something
            kPTurn, kITurn, kDTurn = 0.007, 0.004, 0.00015
        elif moduleName == "rearRight":
            # do something
            kPTurn, kITurn, kDTurn = 0.007, 0.004, 0.00015
            
        self.CPR = 2048
        self.turningGearRatio = 12.8 # The steering motor gear ratio
        self.drivingGearRatio = 8.14 # The driving motor gear ratio
        self.moduleName = moduleName
        self.absoluteOffset = -absoluteOffset
        
        self.driveMotor = ctre.TalonFX(driveID)
        self.turnMotor = ctre.TalonFX(turnID)
        
        self.absoluteEncoder = ctre.CANCoder(absoluteID)
        self.absoluteEncoder.configSensorInitializationStrategy(ctre.SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(ctre.AbsoluteSensorRange.Signed_PlusMinus180)
        self.absoluteEncoder.configMagnetOffset(self.absoluteOffset)
        
        self.turnController = wpimath.controller.PIDController(kPTurn, kITurn, kDTurn)
        self.turnController.enableContinuousInput(-180, 180)
        self.turnController.setTolerance(0.1) # change this number to change accuracy and jitter of motor
        self.moduleReversed = False
        
        self.xDisplacement = 0
        self.yDisplacement = 0
        
        self.initMotorEncoder()
    
    def navxAngleToUnitCircle(self, angle: float):
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        return(angle)
    
    def fixAngleBounds(self, angle: float):
        angle -= 90
        angle += 180
        angle %= 360
        angle -= 180
        return(angle)
    
    def move(self, magnitude: float, angle: float):
        ''' Magnitude with an input range for 0-1, and an angle of -180->180'''
        #angle = self.navxAngleToUnitCircle(angle)
        angle = self.fixAngleBounds(angle)
        motorPosition = self.optimize(angle)
        # print(f"{self.moduleName} Optimize Motor Position: {motorPosition}")
        if self.moduleReversed:
            magnitude = -magnitude
        self.turnController.setSetpoint(angle)
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        self.driveMotor.set(ctre.ControlMode.PercentOutput, magnitude)
        
    def stationary(self):
        ''' Keeps the swerve module still. This implementation is pretty janky tbh '''
        # may need to implement a thing for the turncontroller to still run in here if it had a previous target it never met
        self.driveMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.turnMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.brake()
        
    def coast(self):
        ''' Coasts the swerve module '''
        self.driveMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.turnMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Coast)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Coast)
    
    def brake(self):
        ''' Brakes the swerve module '''
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def initMotorEncoder(self):
        ''' Called to actually set the encoder zero based off of absolute offset and position '''
        self.turnMotor.configIntegratedSensorAbsoluteRange(ctre.AbsoluteSensorRange.Unsigned_0_to_360)
        self.turnMotor.setSelectedSensorPosition(int(self.absoluteEncoder.getAbsolutePosition() * self.CPR * self.turningGearRatio / 360))
        
    def getTurnMotorPosition(self):
        '''motorPosition = ((self.turnMotor.getSelectedSensorPosition(0) % (self.CPR*self.turningGearRatio)) * 360/(self.CPR*self.turningGearRatio))
        if motorPosition > 180:
            motorPosition -= 360'''
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
        if not self.moduleReversed:
            return motorPosition
        else:
            oppositeAngle = motorPosition - 180
            if oppositeAngle < -180:
                oppositeAngle += 360
            return motorPosition
    
    def getDriveMotorVelocity(self):
        ''' Returns the drive motor velocity in meters per second '''
        velocityMPS = self.driveMotor.getSelectedSensorVelocity(0) * 10 * 0.1016 * math.pi / (self.CPR * 8.14)
        '''if self.moduleReversed:
            return -velocityMPS
        else:
            return velocityMPS'''
        return velocityMPS
    
    '''def enableToZero(self):
        motorPosition = self.motorPosition()
        self.turnController.setSetpoint(0) # may have to change this to 180
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)'''
        
    def optimize(self, moduleTarget: float):
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
        normal = abs(motorPosition - moduleTarget)
        if normal > 180:
            normal -= 360
            normal = abs(normal)
        oppositeAngle = motorPosition - 180
        if oppositeAngle < -180:
            oppositeAngle += 360
        opposite = abs(motorPosition - moduleTarget - 180)
        if opposite > 180:
            opposite -= 360
            opposite = abs(opposite)
        # print(f"Opposite: {opposite}, Normal: {normal}, ModuleAngle: {motorPosition}")
        if opposite < normal:
            self.moduleReversed = True
            return oppositeAngle
        else:
            self.moduleReversed = False
            return motorPosition
            
    
    def returnValues(self):
        motorPosition = self.getTurnMotorPosition()
        rawAbsolute = self.absoluteEncoder.getAbsolutePosition() - self.absoluteOffset
        return (motorPosition, self.absoluteEncoder.getAbsolutePosition(), self.absoluteOffset, rawAbsolute) 