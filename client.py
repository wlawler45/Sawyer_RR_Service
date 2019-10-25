import RobotRaconteur as RR
from RobotRaconteur.Client import *
import numpy as np
import time
####################Start Service
#RRN=RR.RobotRaconteurNode.s
#RRN.RegisterServiceTypeFromFile("com.robotraconteur.geometry")
inst=RRN.ConnectService('rr+tcp://localhost:52222/?service=SmartCam')
Sawyer=RRN.ConnectService('tcp://localhost:8884/SawyerJointServer/Sawyer')



destination=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose",Sawyer)

position=np.zeros((1,),dtype=destination)
position[0]['orientation']['w']=0.8470258482846341
position[0]['orientation']['x']=0.5313043122867314
position[0]['orientation']['y']=0.014777107322197721
position[0]['orientation']['z']=0.006676614591848186
position[0]['position']['z']=0.16982119162366985



#########################################calibration
pose=0
def incoming_state(w,state,time):
	global pose
	pose=state.kin_chain_tcp

wire=Sawyer.robot_state.Connect()
wire.WireValueChanged+=incoming_state


# raw_input("please remove robot and place 3 objects")
# inst.update()
# while (len(inst.objects)!=3):
# 	print("number of objects detected: ",len(inst.objects))
# 	raw_input("please remove robot and place 3 objects")
# 	inst.update()

# x=[]
# y=[]
# x_r=[]
# y_r=[]


# for i in range(3):
# 	raw_input("please put robot endeffector on top of object: "+inst.objects[i].name)
# 	x.append(inst.objects[i].x/1000.)
# 	y.append(inst.objects[i].y/1000.)
# 	x_r.append(pose[0]['position']['x'])
# 	y_r.append(pose[0]['position']['y'])
# 	print(x[-1],y[-1],x_r[-1],y_r[-1])


# A=np.array([[x[0],y[0],1],[x[1],y[1],1],[x[2],y[2],1]])
# b=np.array([[x_r[0],y_r[0]],[x_r[1],y_r[1]],[x_r[2],y_r[2]]])

# x = np.linalg.solve(A, b)
# H=np.concatenate((np.transpose(x),np.array([[0,0,1]])),axis=0)

# #############################Imposing constraints
# R=H[:2,:2]
# T=np.transpose(np.array([H[:2,2]]))
# u,s,vh=np.linalg.svd(R)
# R=np.dot(u,vh)
# H=np.concatenate((np.concatenate((R,T),axis=1),np.array([[0,0,1]])),axis=0)
# print(H)



H=np.array([[0.99942127,0.03401641,0.01130381],[-0.03401641,0.99942127,0.05279206],[0,0,1]])


############################Move to Object
inst.update()
p=np.dot(H,np.array([[inst.objects[0].x/1000.],[inst.objects[0].y/1000.],[1]]))
position[0]['position']['x']=p[0]
position[0]['position']['y']=p[1]
velocity=RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialVelocity",Sawyer)
spatial_velocity=RRN.ArrayToNamedArray(np.zeros(6,dtype=float),velocity)
Sawyer.speed_ratio=1
Sawyer.command_mode=3
Sawyer.jog_cartesian({0:position},{0:spatial_velocity},False,False)




# while(1):
# 	inst.update()
# 	print('number:',len(inst.objects))
# 	for i in range(len(inst.objects)):
# 			print('object name:', inst.objects[i].name)
# 			print('object detect:', inst.objects[i].detect)
# 			print('object x:', inst.objects[i].x)
# 			print('object y:', inst.objects[i].y)
# 			print('object angle:', inst.objects[i].angle)
# 			print('object scale:', inst.objects[i].scale)
# 			print('object score:', inst.objects[i].score)
# 	print('\n')
# 	
