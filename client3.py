import RobotRaconteur as RR
from RobotRaconteur.Client import *
import numpy as np
import time
from kinematics import invKine
from FK_UR5 import fwd_kin, inv_kin
from inv_kin import lab_invk
####################Start Service
inst=RRN.ConnectService('rr+tcp://128.113.224.57:52222/?service=SmartCam')
Sawyer=RRN.ConnectService('tcp://128.113.224.58:8884/SawyerJointServer/Sawyer')
UR=RRN.ConnectService('tcp://128.113.224.86:2355/URConnection/Universal_Robot')


destination=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose",Sawyer)
position=np.zeros((1,),dtype=destination)
position[0]['orientation']['w']=0.8470258482846341
position[0]['orientation']['x']=0.5313043122867314
position[0]['orientation']['y']=0.014777107322197721
position[0]['orientation']['z']=0.006676614591848186
position[0]['position']['z']=0.16982119162366985

def move(p,robot):
	global Sawyer, UR, position
	if (robot=="UR"):
		if (p[0]>0):
			p[0]-=0.006
		else:
			p[0]+=0.006
		if (p[1]>0):
			p[1]-=0.006
		else:
			p[1]+=0.006
		temp_q=lab_invk(-p[0],-p[1],p[2],-90)
		UR.jog_joint(temp_q,np.array([0.2,0.2,0.2,0.2,0.2,0.2]),False,False)
	else:
		position[0]['position']['x']=p[0]
		position[0]['position']['y']=p[1]
		position[0]['position']['z']=p[2]
		velocity=RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialVelocity",Sawyer)
		spatial_velocity=RRN.ArrayToNamedArray(np.zeros(6,dtype=float),velocity)
		Sawyer.speed_ratio=1

		Sawyer.command_mode=3
		Sawyer.jog_cartesian({0:position},{0:spatial_velocity},False,False)	
	return





#########################################calibration for Sawyer
# pose=0
# def incoming_state(w,state,time):
# 	global pose
# 	pose=state.kin_chain_tcp

# wire=Sawyer.robot_state.Connect()
# wire.WireValueChanged+=incoming_state


# raw_input("please remove robot and place 3 objects")
# inst.update()
# while (len(inst.objects)<3):
# 	print("number of objects detected: ",len(inst.objects))
# 	raw_input("please remove robot and place 3 objects")
# 	inst.update()

# x=[]
# y=[]
# x_r=[]
# y_r=[]


# for i in range(3):
# 	raw_input("please put robot endeffector on top of object: "+inst.objects[i].name)
	# x.append(inst.objects[i].x/1000.)
	# y.append(inst.objects[i].y/1000.)
	# x_r.append(pose[0]['position']['x'])
	# y_r.append(pose[0]['position']['y'])
	# print(x[-1],y[-1],x_r[-1],y_r[-1])


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



H_Sawyer=np.array([[0.99942127,0.03401641,0.01130381],[-0.03401641,0.99942127,0.05279206],[0,0,1]])
print(H_Sawyer)

#####################################Calibration for UR
# q=0
# def incoming_state(w,state,time):
# 	global q
# 	q=state.joint_position

# wire=UR.robot_state.Connect()
# wire.WireValueChanged+=incoming_state


# input("please remove robot and place 3 objects")


# inst.update()
# while (len(inst.objects)!=3):
# 	print("number of objects detected: ",len(inst.objects))
# 	input("please remove robot and place 3 objects")
# 	inst.update()

# x=[]
# y=[]
# x_r=[]
# y_r=[]


# for i in range(3):
# 	input("please put robot endeffector on top of object: "+inst.objects[i].name)
# 	position=fwd_kin(q)
# 	x.append(inst.objects[i].x/1000.)
# 	y.append(inst.objects[i].y/1000.)
# 	x_R_temp=float(input('x location'))
# 	y_R_temp=float(input('y location'))
# 	x_r.append(x_R_temp)
# 	y_r.append(y_R_temp)
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

H_UR=np.array([[0.04223891, -0.99910754,  0.00630481],[0.99910754,  0.04223891, -1.3026918],[0,0,1]])



# waypoint_UR=[-0.3,-0.3,0.4]
# waypoint_Sawyer=[0.30,-0.39,0.2]
# ##############################split objects
# left=[]
# right=[]
# inst.update()
# for obj in inst.objects:
# 	if obj.x/1000.>0.66:
# 		left.append(obj)
# 	else:
# 		right.append(obj)


# ##############################collaborate move

move(waypoint_UR,"UR")
time.sleep(3)
while left:
	p=np.dot(H_UR,np.array([[left[0].x/1000.],[left[0].y/1000.],[1]]))
	p=[p[0][0]-0.01,p[1][0]-0.03,0.3]
	move(p,"UR")
	time.sleep(3)
	p[2]=-0.04
	move(p,"UR")
	time.sleep(3)
	UR.setf_signal("DO0",1)
	move(waypoint_UR,"UR")
	time.sleep(3)
	UR.setf_signal("DO0",0)
	left.pop(0)
	time.sleep(3)








# move(waypoint_Sawyer,"Sawyer")
# time.sleep(3)
# p=np.dot(H_Sawyer,np.array([[right[0].x/1000.],[right[1].y/1000.],[1]]))
# p=[p[0][0],p[1][0],0.3]
# move(p,"Sawyer")
# time.sleep(3)
# p[2]=-0.04
# move(p,"Sawyer")
# time.sleep(3)
# Sawyer.setf_signal("vacuum",0)
# move(waypoint_Sawyer,"Sawyer")
# time.sleep(3)
# Sawyer.setf_signal("vacuum",0)





















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
	
