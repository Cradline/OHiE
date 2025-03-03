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