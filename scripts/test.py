import numpy as np
from math import *
import pandas as pd

import sys

DATA_PATH = '/home/robolab/tello_drone/src/tello_drone_control/Data/'
c = 'drone_1'

data = pd.read_csv(DATA_PATH + c +'.csv')

for i in range(len(data.x)):
    print (data.x[i])

for row in range(len(data)):
    print (row)

k = 0

while k < len(data):
    print (data.x[k])
    k += 1
