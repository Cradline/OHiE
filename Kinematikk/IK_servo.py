#!/usr/bin/env python3
# encoding:utf-8
import sys
import time
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from kinematics.IK_geo import *
from kinematics.transform import getAngle
from mpl_toolkits.mplot3d import Axes3D
import common.yaml_handle as yaml_handle

# Henter avviksdata for servo robotarm
deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)

# Vinkler regnes ut i IK klassen fra IK_geo
ik = IK('arm')

# Angir lenge på lenker
l1 = ik.l1
l4 = ik.l4
ik.setLinkLength(L1=l1+1.3, L4=l4)

# Klasse for servokontroll
class ArmIK:
    # Pulsbredde intervall [us], vinkelintervall [deg]
    servo3Range = (500, 2500.0, 0, 180.0)       # Servo ID3: ledd C
    servo4Range = (500, 2500.0, 0, 180.0)       # Servo ID4: ledd B
    servo5Range = (500, 2500.0, 0, 180.0)       # Servo ID5: ledd A
    servo6Range = (500, 2500.0, 0, 180.0)       # Servo ID6: Base-ledd

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # Angir intervall for servos
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range

        # Regner ut pulsbredde per grad i [us/deg]
        self.servo3Param = ((self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2]))
        self.servo4Param = ((self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2]))
        self.servo5Param = ((self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2]))
        self.servo6Param = ((self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2]))

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # Konverterer vinkler fra IK_geo til korresponderende pulslengder
        # Inkludert mapping fra [0,180] til [-90,90]

        # Vinkel [deg] * 11.11 [us/deg] + (2500 + 500)/2 [us]
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0]:
            # Sjekker om vinkelutslag er innenfor gyldig område
            logger.info('servo3(%s)Utenfor gyldig område(%s, %s)', servo3, self.servo3Range[0], self.servo3Range[1])
            return False
        
        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            # Sjekker om vinkelutslag er innenfor gyldig område
            logger.info('servo4(%s)Utenfor gyldig område(%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False
        
        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 + (90.0 - theta5) * self.servo5Param)) 
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0])/2 + 90*self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0])/2 - 90*self.servo5Param):
            # Sjekker om vinkelutslag er innenfor gyldig område
            # -> Vinkel mellom [-pi/2,pi/2] ? 
            logger.info('servo5(%s)Utenfor gyldig område(%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False
        
        # Konverterer til +deg hvis deg < -90. Med eller mot klokka
        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            # Hvis theta6 < -90 [deg] -> servo6 = (180-0)/2 + 90 + (180 + theta6)
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            # Hvis ikke: servo6 = ((180-0)/2 -(90-theta6))* 11.11us + 500us 
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param)) + self.servo6Range[0]
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            # Sjekker om vinkelutslag er innenfor gyldig område
            logger.info('servo6(%s)Utenfor gyldig område(%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False
        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def servosMove(self, servos, movetime=None):
        # Styrer servoene til de beregnede posisjonene
        if movetime is None:
            max_d = 0
            for i in  range(0, 4):
                #d = abs(board.pwm_servo_read_position(i + 3) - servos[i])
                d = abs(deviation_data['{}'.format(i+3)])
                print(d)
                if d > max_d:
                    max_d = d
            movetime = int(max_d*1)
        self.board.pwm_servo_set_position(float(movetime)/1000.0,[[3,servos[0]+deviation_data['3']]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0,[[4,servos[1]+deviation_data['4']]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0,[[5,servos[2]+deviation_data['5']]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0,[[6,servos[3]+deviation_data['6']]])

        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da = 1):
        # Flytter robotarmen til en posisjon med en gitt pitch-vinkel
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):#
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']               
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha

        return False


    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime = None):

# Gitt koordinater coordinate_data og pitch-vinkel alpha, samt pitch-intervallet alpha1 og alpha2,
# automatisk finn den løsningen som er nærmest den gitte pitch-vinkelen og flytt til målposisjonen.
# Hvis det ikke finnes noen løsning, returner False. Ellers returner servo-vinkler, pitch-vinkel og kjøretid.
# Koordinatene er i centimeter og sendes som en tuppel, for eksempel (0, 5, 10).
# alpha er den gitte pitch-vinkelen.
# alpha1 og alpha2 er området for pitch-vinkelen.
# movetime er tiden det tar for servoen å bevege seg, i millisekunder. Hvis tiden ikke er oppgitt, beregnes den automatisk.
        
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]
        movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)
        return servos, alpha, movetime
 
if __name__ == "__main__":
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    #print(AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500))