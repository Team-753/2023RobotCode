import math

class Arm:
    
    # all of the following setters will be the default arm state measured in radians counter-clockwise from the +x axis
    '''thetaOne = math.pi / 2
    thetaTwo = -math.pi / 4
    thetaThree = 0'''
    
    def __init__(self, l1, l2, l3) -> None:
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        super().__init__()
        
    '''def getCurrentEndEffectorPosition(self):
        x = self.l1 * math.cos(self.thetaOne) + self.l2 * math.cos(self.thetaOne + self.thetaThree) + self.l3 * math.cos(self.thetaOne + self.thetaThree + self.thetaThree)
        y = self.l1 * math.sin(self.thetaOne) + self.l2 * math.sin(self.thetaOne + self.thetaThree) + self.l3 * math.sin(self.thetaOne + self.thetaThree + self.thetaThree)
        return (x, y)'''
    
    def solveInverseKinematics(self, x, y):
        # figuring out straight line distance between the tip of the third segment and the arm base
        d = math.sqrt(x*x + y*y)
        print(d)
        
        # seeing if we can even reach it...
        if (d > self.l1 + self.l2 + self.l3 or d < abs(self.l1 - self.l2 - self.l3)):
        # Position is not reachable, return without updating joint angles
            return False, 0, 0, 0
        
        # Calculate the angle between the base and the end effector
        phi = math.atan2(y, x)
        
        # Calculate the angle between the base and the intersection of the two circles
        gamma = math.acos((self.l1*self.l1 + d*d - self.l2*self.l2) / (2*self.l1*d))
        
        # Calculate theta1
        thetaOneT = phi - gamma
        
        '''if thetaOneT < 0:
            return False, 0, 0, 0'''
        
        # Calculate theta2
        thetaTwoT = math.acos((self.l1*self.l1 + self.l2*self.l2 - d*d) / (2*self.l1*self.l2))
        
        # Calculate theta3
        thetaThreeT = math.atan2(y - self.l1*math.sin(thetaOneT) - self.l2*math.sin(thetaOneT + thetaTwoT), x - self.l1*math.cos(thetaOneT) - self.l2*math.cos(thetaOneT + thetaTwoT))
        
        return True, thetaOneT, thetaTwoT, thetaThreeT
    
    def givethAnglesThyMother(self, tx, ty):
        return self.solveInverseKinematics(tx, ty)

l1 = 24
l2 = 24
l3 = 4

arm = Arm(l1, l2, l3)
result = arm.givethAnglesThyMother(30, 30)
print(f"Possible?: {result[0]}, A1: {math.degrees(result[1])}, A2: {math.degrees(result[2])}, A3: {math.degrees(result[3])}")