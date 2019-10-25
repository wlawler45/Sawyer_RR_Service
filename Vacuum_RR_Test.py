import RobotRaconteur as RR
from RobotRaconteur.Client import *
import numpy as np
import time

Sawyer=RRN.ConnectService('rr+tcp://[fe80::23b1:8dea:d94b:ab27]:8884/?nodeid=e37e7c86-c69c-4075-9ffb-91817e53ec0f&service=Sawyer')

Sawyer.setf_signal("vacuum",0)

