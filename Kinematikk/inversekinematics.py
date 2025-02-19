#!/usr/bin/env python3
# encoding: utf-8
# Inverse kinematics of a 4-DOF robot: Given the corresponding coordinates (X, Y, Z) and pitch angle,
# calculates the rotation angle of each joint.


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
    l4 = 10.00      #Placeholder: 10cm Servo 4 til klo tip

    # Parameters specific to the pneumatic pump version.
    #l5 = 4.70  #Distance from the fourth servo to directly above the suction nozzle: 4.70 cm.
    #l6 = 4.46  #Distance from directly above the suction nozzle to the suction nozzle: 4.46 cm.
    #alpha = degrees(atan(l6 / l5))  #Calculates the angle between l5 and l4

    def __init__(self, arm_type): # Revolusjons eller prismatisk ende-effektor
        self.arm_type = arm_type
        #if self.arm_type == 'pump': # Trolig unødvendig
        #    self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))  # unødvendig
        #elif self.arm_type == 'arm':
        #    self.l4 = 10.00  #The distance from the fourth servo to the end of the robotic arm is 16.6cm? Claw closed.
        self.arm_type == 'arm'

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):#, L5=l5, L6=l6):
        # Change the link lengths of the robotic arm to adapt to arms of the same structure but different lengths.
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        #self.l5 = L5
        #self.l6 = L6
        #if self.arm_type == 'pump':                             # Trolig mulig 
        #    self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
        #    self.alpha = degrees(atan(self.l6 / self.l5))

    def getLinkLength(self):
        # Get the currently set link lengths.
        #if self.arm_type == 'pump':
        #    return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        #else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # Given the specified coordinates and pitch angle, return the angles each joint should rotate. If no solution is found, return False.
        # coordinate_data is the coordinates of the end effector P_E/P, in centimeters (cm), passed as a tuple, e.g; (0, 5, 10).
        # Alpha is the angle between the end effector and the horizontal plane, in degrees.

        # Point P(X,Y,Z) is the end effector (P_E). O is the Origin projected on the ground. P is the projection of point P(X,Y,Z) on the ground.
        # Point A is the intersection between l1 and l2. Point B is the intersection between l2 and l3. Point C is the intersection between l3 and l4.
        # CD = vinkelrett på PD og Z-aksen. Pitch angle Alpha er vinkel mellom CD og PC. 
        # Vinkler er representert på følgende måte: Vinkel mellom AB og BC er ABC

        X, Y, Z = coordinate_data
        #if self.arm_type == 'pump':
        #    Alpha -= self.alpha

        # Theta_6 = base rotation angle
        theta6 = degrees(atan2(Y, X))           # den vi tegne ovenfra som theta_1, base-servo: rotasjon rundt Z_0
 
        P_O = sqrt(X*X + Y*Y)                   # distanse fra Origo til ende-effektor
        CD = self.l4 * cos(radians(Alpha))      # Lengde (X) fra ledd-C til ende-effektor (hjelpe-punkt D)
        PD = self.l4 * sin(radians(Alpha))      # Høyde (Z) fra ledd-C til ende-effektor, (hjelpe-punkt D). Alpha pos -> PD = pos, Alpha neg -> PD = neg.
        AF = P_O - CD                           # distanse fra Origo (eller A) til hjelpe-punkt F, langs X i XZ.  
        CF = Z - self.l1 - PD                   # Høyde (Z) mellom C og hjelpe-punkt F
        AC = sqrt(pow(AF, 2) + pow(CF, 2))      # distance mellom punkt A og C, via hjelpe-punkt F

        if round(CF, 4) < -self.l1:
            logger.debug('Height below 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # The sum of two sides is less than the third side
            logger.debug('Cannot establish linkage mechanism, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        # Theta_4. phi_B = vinkel ABC, mellom AB og BC
        cos_phi_B = round((pow(self.l2, 2) + pow(self.l3, 2) - pow(AC, 2))/(2*self.l2*self.l3), 4) # Law of cosines
        if abs(phi_B) > 1:
            logger.debug('Cannot establish linkage mechanism, abs(cos_phi_B(%s)) > 1', cos_phi_B)
            return False
        phi_B = acos(cos_phi_B)             # Finner vinkel phi_B i [rad]
        theta4 = 180.0 - degrees(phi_B)     # konverterer til grader

        # Theta_5. 
        CAF = acos(AF / AC)         # CAF = vinkel mellom AF og CF. 
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) # Law of cosines
        if abs(cos_BAC) > 1:
            logger.debug('Cannot establish linkage mechanism, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:                  # Hvis CF er negativ, betyr det at punkt F er over punkt C. zf flag settes til negativ for å justere vinkel CAF
            zf_flag = -1            # Hvis F er under punkt C, er CAF positiv
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        # Theta_3
        theta3 = Alpha - theta5 + theta4
        #if self.arm_type == 'pump':     # Trolig unødvendig og kan slettes
        #    theta3 += self.alpha        # same

        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # Returns the angles if there is a solution
            
if __name__ == '__main__':
    ik = IK('arm')
    #ik.setLinkLength(L1=ik.l1 + 1.30, L4=ik.l4)
    print('Link length：', ik.getLinkLength())
    #print(ik.getRotationAngle((0, ik.l4, ik.l1 + ik.l2 + ik.l3), 0))
