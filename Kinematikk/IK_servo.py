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
    servo3Range = (500, 2500.0, 0, 180.0)
    servo4Range = (500, 2500.0, 0, 180.0)
    servo5Range = (500, 2500.0, 0, 180.0)
    servo6Range = (500, 2500.0, 0, 180.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # Angir intervall for servos
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range

