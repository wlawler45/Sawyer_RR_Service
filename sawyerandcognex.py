from RobotRaconteur.Client import *
import time
import numpy as np
inst=RRN.ConnectService('rr+tcp://localhost:52222/?service=Create')
sawyer=RRN.ConnectService('rr+tcp://[fe80::6fd7:46d9:60d1:7d1b]:8884/?nodeid=e37e7c86-c69c-4075-9ffb-91817e53ec0f&service=Sawyer')
orientation_or=[0.8470258482846341,0.5313043122867314,0.014777107322197721,-0.006676614591848186]
orientation=np.array(orientation_or)
position_z=0.019205198462994226
sawyer.easy_mode=3


while(1):
    inst.update_objects()
    print('number:',len(inst.objects))
    for i in range(len(inst.objects)):
        if(inst.objects[i].name=="L"):
            print("I Saw an L")
            print('object detect:', inst.objects[i].detect)
            x=inst.objects[i].x/1000
            y=inst.objects[i].y/1000
            print('object x:', x)
            print('object y:', y)
            print('object angle:', inst.objects[i].angle)
            print('object score:', inst.objects[i].score)
            
            position_or=[x,y,position_z]
            position=np.array(position_or)
            sawyer.easy_jog_cartesian(position,orientation)
    print('\n')
    time.sleep(0.5)
    #Quaternion(x=0.8470258482846341, y=0.5313043122867314, z=0.014777107322197721, w=-0.006676614591848186)


