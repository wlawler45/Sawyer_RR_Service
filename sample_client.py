from RobotRaconteur.Client import *
import time
import numpy
import sys


def main():

    url='rr+tcp://[fe80::6fd7:46d9:60d1:7d1b]:2355/?nodeid=0912818c-8e8f-4022-b31a-63efdc64ad49&service=Universal_Robot'

    
    
    #Connect to the service
    c=RRN.ConnectService(url)
    c.speed_ratio=1
    c.command_mode=3
    #Start streaming data packets
    #c.easy_mode=3
    c.jog_joint(numpy.array([1.0,1.0,1.0,1.0,1.0,1.0]),numpy.zeros(6,dtype=float),False,False)
    wire=c.robot_state.Connect()
    wire.WireValueChanged+=incoming_state
 #   position_wire=c.easy_position_command.Connect()
 #   for x in range(0,5,0.1):
  #      position_wire.O
    raw_input("press enter to quit...\r\n")
    #Add a function handler for the "Bump" event
    
def incoming_state(w,state,time):
    print("CURRENT ROBOT STATE:")
    #print("Sequence Number: " +str(state.seqno))
    #print("Mode: "+str(state.controller_mode))
    print("Joint Position: "+str(state.joint_position))
    #print("Joint Velocity: "+str(state.joint_velocity))
    #print("Joint Effort: "+str(state.joint_effort)+"\n")
    print("Joint position:"+str(state.kin_chain_tcp))
    #print("Position Command: "+str(state.position_command))
    #print("Velocity Command: "+str(state.velocity_command)+"\n")
    print("----------------------------------------\n")

if __name__ == '__main__':
    main()
