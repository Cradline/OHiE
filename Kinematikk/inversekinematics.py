#!/usr/bin/env python3
# encoding: utf-8
# 4自由度机械臂逆运动学：给定相应的坐标（X,Y,Z），以及俯仰角，计算出每个关节转动的角度
# 2020/07/20 Aiden
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # Servos are counted from bottom to top.
    # Link parameters of the 4-DOF robotic arm.
    l1 = 8.00       #Distance from the center of the robotic arm base to the axis of the second servo: 6.10 cm.
    l2 = 6.50       #Distance from the second servo to the third servo: 10.16 cm.
    l3 = 6.20       #Distance from the third servo to the fourth servo: 9.64 cm.
    l4 = 0.00       #这里不做具体赋值，根据初始化时的选择进行重新赋值

    # Parameters specific to the pneumatic pump version.
    l5 = 4.70  #Distance from the fourth servo to directly above the suction nozzle: 4.70 cm.
    l6 = 4.46  #Distance from directly above the suction nozzle to the suction nozzle: 4.46 cm.
    alpha = degrees(atan(l6 / l5))  #Calculates the angle between l5 and l4

    def __init__(self, arm_type): # Revolusjons eller prismatisk ende-effektor
        self.arm_type = arm_type
        if self.arm_type == 'pump': #如果是气泵款机械臂
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))  #第四个舵机到吸嘴作为第四个连杆
        elif self.arm_type == 'arm':
            self.l4 = 10.00  #第四个舵机到机械臂末端的距离16.6cm， 机械臂末端是指爪子完全闭合时

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4, L5=l5, L6=l6):
        # Change the link lengths of the robotic arm to adapt to arms of the same structure but different lengths.
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        self.l5 = L5
        self.l6 = L6
        if self.arm_type == 'pump':
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
            self.alpha = degrees(atan(self.l6 / self.l5))

    def getLinkLength(self):
        # Get the currently set link lengths.
        if self.arm_type == 'pump':
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # Given the specified coordinates and pitch angle, return the angles each joint should rotate. If no solution is found, return False.
        # coordinate_data is the coordinates of the end effector, in centimeters (cm), passed as a tuple, e.g., (0, 5, 10).
        # Alpha is the angle between the end effector and the horizontal plane, in degrees.

        # Point P(X,Y,Z) is the end effector (P_E). O is the Origin projected on the ground. P is the projection of point P(X,Y,Z) on the ground.
        # Point A is the intersection between l1 and l2. Point B is the intersection between l2 and l3. Point C is the intersection between l3 and l4.
        # CD = vinkelrett på PD og Z-aksen. Pitch angle Alpha er vinkel mellom CD og PC. 
        # Vinkler er representert på følgende måte: Vinkel mellom AB og BC er ABC

        X, Y, Z = coordinate_data
        if self.arm_type == 'pump':
            Alpha -= self.alpha
        # Theta6 = base rotation angle
        theta6 = degrees(atan2(Y, X))           # den vi tegne ovenfra som theta_1, base-servo: rotasjon rundt Z_0
 
        P_O = sqrt(X*X + Y*Y)                   # distanse fra Origo til ende-effektor
        CD = self.l4 * cos(radians(Alpha))      # Lengde (X) fra ledd-C til ende-effektor (hjelpe-punkt D)
        PD = self.l4 * sin(radians(Alpha))      # Høyde (Z) fra ledd-C til ende-effektor (hjelpe-punkt D). Alpha pos -> PD = pos, Alpha neg -> PD = neg.
        AF = P_O - CD                           # distanse fra Origo (eller A) til hjelpe-punkt F, langs X i XZ.  
        CF = Z - self.l1 - PD                   # Høyde (Z) mellom C og hjelpe-punkt F
        AC = sqrt(pow(AF, 2) + pow(CF, 2))      # distance mellom punkt A og C, via hjelpe-punkt F
        if round(CF, 4) < -self.l1:
            logger.debug('高度低于0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # The sum of two sides is less than the third side
            logger.debug('Cannot establish linkage mechanism, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        #求theat4
        cos_ABC = round((pow(self.l2, 2) + pow(self.l3, 2) - pow(AC, 2))/(2*self.l2*self.l3), 4) # Law of cosines
        if abs(cos_ABC) > 1:
            logger.debug('Cannot establish linkage mechanism, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) #反三角算出弧度
        theta4 = 180.0 - degrees(ABC)

        #求theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) # Law of cosines
        if abs(cos_BAC) > 1:
            logger.debug('Cannot establish linkage mechanism, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        #求theta3
        theta3 = Alpha - theta5 + theta4
        if self.arm_type == 'pump':
            theta3 += self.alpha

        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # 有解时返回角度字典
            
if __name__ == '__main__':
    ik = IK('arm')
    #ik.setLinkLength(L1=ik.l1 + 1.30, L4=ik.l4)
    print('连杆长度：', ik.getLinkLength())
    #print(ik.getRotationAngle((0, ik.l4, ik.l1 + ik.l2 + ik.l3), 0))
