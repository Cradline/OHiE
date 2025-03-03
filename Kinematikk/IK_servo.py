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
        # Vinkel [deg] * 11.11 [us/deg] + 2500[us] + (500/2)[us]
        
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
        
