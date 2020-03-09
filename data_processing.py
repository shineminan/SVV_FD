import numpy as np
import scipy.io
import matplotlib.pyplot as plt

'''
1        Angle of attack
2        Deflection of elevator trim
3        Force on elevator control wheel
4        Engine 1: Fuel mass flow
5        Engine 2: Fuel mass flow
6        Engine 1: Inter Turbine Temperature (ITT)
7        Engine 2: Inter turbine temperature (ITT)
8        Engine 1: Oil pressure
9        Engine 2: Oil pressure
10       Deflection of the control column (Se or DCOC)
11       Engine 1: Fan speed (N1)
12       Engine 1: Turbine speed (N2)
13       Engine 2: Fan speed (N1)
14       Engine 2: Turbine speed (N2)
15       calculated fuel used by fuel mass flow, left
16       calculated fuel used by fuel mass flow, right
17       Deflection of aileron (right wing?)
18       Deflection of elevator
19       Deflection of rudder
20       UTC Date DD:MM:YY
21       UTC Seconds
22       Roll Angle
23       Pitch Angle
24       Fms1_trueHeading
25       GNSS Latitude
26       GNSS Longitude
27       Body Roll Rate
28       Body Pitch Rate
29       Body Yaw Rate
30       Body Long Accel
31       Body Lat Accel
32       Body Norm Accel
33       Along Heading Accel
34       Cross Heading Accel
35       Vertical Accel
36       Static Air Temperature
37       Total Air Temperature
38       Pressure Altitude (1013.25 mB)
39       Baro Corrected Altitude #1
40       Dadc1_bcAltMb
41       Mach
42       Computed Airspeed
43       True Airspeed
44       Altitude Rate
45       Measurement Running
46       Number of Measurements Ready
47       Status of graph
48       Active Screen
49       Time
'''

# Read data from .mat
test_data = scipy.io.loadmat('referencedata.mat')['flightdata'][0][0]
def mat_data(i, item=0):
    entry = test_data[i][0][0]
    if item==0:
        return entry[item].flatten()
    elif item==1 or item==2:
        return entry[item][0][0][0]
    else:
        return 'FuckYou!'


print(mat_data(3))

