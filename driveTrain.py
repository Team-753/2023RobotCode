from swerveModule import swerveModule
import math
import wpilib

class driveTrain:
    def __init__(self, config: dict, navxOBJ: object):
        self.navx = navxOBJ
        self.swerveModules = {
            "frontLeft": None,
            "frontRight": None,
            "rearLeft": None,
            "rearRight": None
        }
        self.config = config
        self.fieldOrient = bool(self.config["RobotDefaultSettings"]["fieldOrient"])
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        # LConfig = self.config["SwerveModules"]["frontLeft"]
        for i in range(4):
            moduleName = list(self.config["SwerveModules"])[i]
            swerveConfig = self.config["SwerveModules"][moduleName]
            self.swerveModules[moduleName] = swerveModule(swerveConfig["motor_ID_1"], swerveConfig["motor_ID_2"], swerveConfig["encoder_ID"], swerveConfig["encoderOffset"], moduleName)
            self.swerveModules[moduleName].initMotorEncoder()


    def rotateCartesianPlane(self, angle: float, x: float, y: float):
        newX = x*math.sin(angle) - y*math.cos(angle)
        newY = x*math.cos(angle) + y*math.sin(angle)
        return(newX, newY)

    def move(self, joystickX: float, joystickY: float, joystickRotation: float):
        '''
        This method takes the joystick inputs from the driverStation class. 
        First checking to see if it is field oriented and compensating for the navx angle if it is.
        NOTE: The final angle may be in unit circle degrees and not in normal oriented degrees this is most likely the problem if the drivetrain has a 90 degree offset
        '''
        joystickX, joystickY = joystickX * self.swerveSpeedFactor, joystickY * self.swerveSpeedFactor
        
        if self.fieldOrient:
            angle %= 360
            if angle < -180:
                angle += 360
            elif angle > 180:
                angle -= 360
            angleRadians = angle*math.pi/180
            translationVector = self.rotateCartesianPlane(angleRadians, joystickX, joystickY)
        else:
            translationVector = (joystickX, joystickY)
            
        fLRotationVector = (joystickRotation*math.cos(self.fLRotationVectorAngle), joystickRotation*math.sin(self.fLRotationVectorAngle))
        fRRotationVector = (joystickRotation*math.cos(self.fRRotationVectorAngle), joystickRotation*math.sin(self.fRRotationVectorAngle))
        rLRotationVector = (joystickRotation*math.cos(self.rLRotationVectorAngle), joystickRotation*math.sin(self.rLRotationVectorAngle))
        rRRotationVector = (joystickRotation*math.cos(self.rRRotationVectorAngle), joystickRotation*math.sin(self.rRRotationVectorAngle))
        
        fLTranslationVector = (fLRotationVector[0] + translationVector[0], fLRotationVector[1] + translationVector[1])
        fRTranslationVector = (fRRotationVector[0] + translationVector[0], fRRotationVector[1] + translationVector[1])
        rLTranslationVector = (rLRotationVector[0] + translationVector[0], rLRotationVector[1] + translationVector[1])
        rRTranslationVector = (rRRotationVector[0] + translationVector[0], rRRotationVector[1] + translationVector[1])
        
        fLAngle = math.atan2(fLTranslationVector[1], fLTranslationVector[0])*180/math.pi
        fRAngle = math.atan2(fRTranslationVector[1], fRTranslationVector[0])*180/math.pi
        rLAngle = math.atan2(rLTranslationVector[1], rLTranslationVector[0])*180/math.pi
        rRAngle = math.atan2(rRTranslationVector[1], rRTranslationVector[0])*180/math.pi

        fLSpeed = math.sqrt((fLTranslationVector[0]**2) + (fLTranslationVector[1]**2))
        fRSpeed = math.sqrt((fRTranslationVector[0]**2) + (fRTranslationVector[1]**2))
        rLSpeed = math.sqrt((rLTranslationVector[0]**2) + (rLTranslationVector[1]**2))
        rRSpeed = math.sqrt((rRTranslationVector[0]**2) + (rRTranslationVector[1]**2))

        maxSpeed = max(fLSpeed, fRSpeed, rLSpeed, rRSpeed)
        if maxSpeed > 1:
            fLSpeed /= maxSpeed
            fRSpeed /= maxSpeed
            rLSpeed /= maxSpeed
            rRSpeed /= maxSpeed
        self.swerveModules["frontLeft"].move(fLSpeed, fLAngle)
        self.swerveModules["frontRight"].move(fRSpeed, fRAngle)
        self.swerveModules["rearLeft"].move(rLSpeed, rLAngle)
        self.swerveModules["rearRight"].move(rRSpeed, rRAngle)

    def reInitiateMotorEncoders(self):
        ''' Call this when actually re-zeroing the motor absolutes '''
        self.swerveModules["frontLeft"].initMotorEncoder()
        self.swerveModules["frontRight"].initMotorEncoder()
        self.swerveModules["rearLeft"].initMotorEncoder()
        self.swerveModules["rearRight"].initMotorEncoder()
        
    def stationary(self):
        ''' Makes the robot's drivetrain stationary '''
        self.swerveModules["frontLeft"].stationary()
        self.swerveModules["frontRight"].stationary()
        self.swerveModules["rearLeft"].stationary()
        self.swerveModules["rearRight"].stationary()

    def coast(self):
        ''' Coasts the robot's drivetrain '''
        self.swerveModules["frontLeft"].coast()
        self.swerveModules["frontRight"].coast()
        self.swerveModules["rearLeft"].coast()
        self.swerveModules["rearRight"].coast()
        
    def refreshValues(self):
        '''frontLeftValues = self.swerveModules["frontLeft"].returnValues()
        frontRightValues = self.swerveModules["frontRight"].returnValues()
        rearLeftValues = self.swerveModules["rearLeft"].returnValues()
        rearRightValues = self.swerveModules["rearRight"].returnValues()
        return frontLeftValues, frontRightValues, rearLeftValues, rearRightValues'''
        pass