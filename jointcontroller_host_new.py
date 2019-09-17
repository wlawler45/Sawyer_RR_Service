#!/usr/bin/env python
#import roslib
#roslib.load_manifest('sawyer_rr_bridge')
import rospy
import intera_interface
from std_msgs.msg import Empty

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
RRN=RR.RobotRaconteurNode.s
sawyer_servicedef="""
#Service to provide simple interface to Sawyer
service SawyerJoint_Interface

option version 0.4

object Sawyer
property double[] joint_positions
property double[] joint_velocities
property double[] joint_torques
property double[] endeffector_positions
property double[] endeffector_orientations
property double[] endeffector_twists
property double[] endeffector_wrenches

function void setControlMode(uint8 mode)
function void setJointCommand(string limb, double[] command)
function void setPositionModeSpeed(double speed)
function void setNeutral()
end object

"""
class Sawyer_impl(object):
    def __init__(self):
        print("Initializing Node")
        rospy.init_node('sawyer_jointstates')
        
        print("Enabling Robot")
        rs = intera_interface.RobotEnable()
        rs.enable()        
        self.seqno=0
        self._valid_limb_names = {'right': 'right',
                                    'r': 'right'}
        
        # get information from the SDK
        self._right = intera_interface.limb.Limb("right")
        self._r_jnames = self._right.joint_names()
        self._lock=threading.RLock()
        # data initializations
        
        self._jointpos = numpy.zeros(7,dtype=float)
        self._jointvel = numpy.zeros(7,dtype=float)
        self._jointtor = numpy.zeros(7,dtype=float)
        self._ee_pos = numpy.zeros(3,dtype=float)
        self._ee_or = numpy.zeros(4,dtype=float)
        self._ee_tw = numpy.zeros(6,dtype=float)
        self._ee_wr = numpy.zeros(6,dtype=float)
        self._r_joint_command = dict(zip(self._r_jnames,[0.0]*7))
        self.MODE_HALT=0
        self.MODE_POSITION = 3#;#1
        self.MODE_VELOCITY = 4#;#2
        self.MODE_TORQUE = 1#;#3
        self.MODE_TRAJECTORY = 2#;#4
        self._downsample=0.01
        self._mode = self.MODE_HALT
        self._trajectory_running=False
        self._robot_state_sensor_data=None
        self._joint_command=numpy.zeros(7,dtype=float)
        self._velocity_command=numpy.zeros(7,dtype=float)
        # initial joint command is current pose
        self.readJointPositions()
        self.setJointCommand('right',self._jointpos[0:7])
                
        # Start background threads
        self._jp_pub = False
        self._running = False
        

    def close(self):
        self._running = False
        self._t_joints.join()
        self._t_effector.join()
        self._t_command.join()
        
        if (self._mode != self.MODE_POSITION):
            self._right.exit_control_mode()

    def start(self):
        self._running=True
        
    
        self._t_joints = threading.Thread(target=self.jointspace_worker)
        self._t_joints.daemon = True
        self._t_joints.start()
        
        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()
        
    def setNeutral(self):
        self._jp_pub = False;
        print("Moving to neutral pose...")
        self._right.move_to_neutral()
    """
    @property	
    def joint_positions(self):
    
        return self._jointpos
    
    @property	
    def joint_velocities(self):
        return self._jointvel

    @property	
    def joint_torques(self):
        return self._jointtor
    
    @property 
    def endeffector_positions(self):
        return self._ee_pos
    
    @property 
    def endeffector_orientations(self):
        return self._ee_or
    
    @property 
    def endeffector_twists(self):
        return self._ee_tw
    
    @property 
    def endeffector_wrenches(self):
        return self._ee_wr
    """
    def readJointPositions(self):
        r_angles = self._right.joint_angles()        
        if r_angles:
            for i in xrange(0,len(self._r_jnames)):
                self._jointpos[i] = r_angles[self._r_jnames[i]]
            return numpy.asarray(self._jointpos)
            
    def readJointVelocities(self):
        r_velocities = self._right.joint_velocities()
        if r_velocities:
            for i in xrange(0,len(self._r_jnames)):
                self._jointvel[i] = r_velocities[self._r_jnames[i]]
            return numpy.asarray(self._jointvel)
            
    def readJointTorques(self):
        r_efforts = self._right.joint_efforts()
        if r_efforts:
            for i in xrange(0,len(self._r_jnames)):
                self._jointtor[i] = r_efforts[self._r_jnames[i]]
            return numpy.asarray(self._jointtor)
    
    def readEndEffectorPoses(self):
        r_pose = self._right.endpoint_pose()
        if r_pose:
            self._ee_pos[0] = r_pose['position'].x
            self._ee_pos[1] = r_pose['position'].y
            self._ee_pos[2] = r_pose['position'].z
            self._ee_or[0] = r_pose['orientation'].w
            self._ee_or[1] = r_pose['orientation'].x
            self._ee_or[2] = r_pose['orientation'].y
            self._ee_or[3] = r_pose['orientation'].z
        
    def readEndEffectorTwists(self):
        r_twist = self._right.endpoint_velocity()
        if r_twist:
            self._ee_tw[0] = r_twist['angular'].x
            self._ee_tw[1] = r_twist['angular'].y
            self._ee_tw[2] = r_twist['angular'].z
            self._ee_tw[3] = r_twist['linear'].x
            self._ee_tw[4] = r_twist['linear'].y
            self._ee_tw[5] = r_twist['linear'].z
    def readEndEffectorWrenches(self):
        r_wrench = self._right.endpoint_effort()
        if r_wrench:
            self._ee_wr[0] = r_wrench['torque'].x
            self._ee_wr[1] = r_wrench['torque'].y
            self._ee_wr[2] = r_wrench['torque'].z
            self._ee_wr[3] = r_wrench['force'].x
            self._ee_wr[4] = r_wrench['force'].y
            self._ee_wr[5] = r_wrench['force'].z
    
    
    def setControlMode(self, mode):
        if mode != self.MODE_POSITION and \
                mode != self.MODE_VELOCITY and \
                mode != self.MODE_TORQUE:
            return
        if mode == self.MODE_POSITION:
            self._right.exit_control_mode()
            # set command to current joint positions
            self.setJointCommand('right',self._jointpos[0:7])
        elif mode == self.MODE_VELOCITY:
            # set command to zeros
            self.setJointCommand('right',numpy.zeros((1,7),dtype=float))
        elif mode == self.MODE_TORQUE:
            # set command to zeros
            self.setJointCommand('right',numpy.zeros((1,7),dtype=float))
        
        self._mode = mode

    def setJointCommand(self, limb, command):
        if not limb in self._valid_limb_names.keys():
            return
        self._jp_pub = True;
        if self._valid_limb_names[limb] == 'right':
            self._joint_command=numpy.asarray(command)
            for i in xrange(0,len(self._r_jnames)):
                self._r_joint_command[self._r_jnames[i]] = command[i]
    
    def setPositionModeSpeed(self, speed):
        if speed < 0.0:
            speed = 0.0
        elif speed > 1.0:
            speed = 1.0
        
        self._right.set_joint_position_speed(speed)
        
    
    # worker function to request and update joint data for sawyer
    # maintain 100 Hz read rate
    # TODO: INCORPORATE USER-DEFINED JOINT PUBLISH RATE
    
    def _execute_joint_command(self, command):
        self._joint_command=numpy.asarray(command)
        for i in xrange(0,len(self._r_jnames)):
            self._r_joint_command[self._r_jnames[i]] = command[i]
    
    def command_cb(self,wire,value,time):
        
        if(wire==self.easy_position_command and self._mode==self.MODE_POSITION):
            self._joint_command=wire.InValue
            self._execute_joint_command(joint_command.command)
        if(wire==self.easy_velocity_command and self._mode==self.MODE_VELOCITY):
            self._velocity_command=wire.InValue
        
        
    def easy_jog(self, joint_positions):
        self.setJointCommand('right',joint_positions)

    def jointspace_worker(self):
        while self._running:
            #statesensordata=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotStateSensorData")
            with self._lock:
                state=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotState")
                #mode=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotMode")
                
                #state.seqno=self.seqno
                state.seqno=self.seqno
                
                print(self._mode)
                state.mode=self._mode
                """
                print(self.readJointPositions())
                print(self.readJointVelocities())
                print(self.readJointTorques())
                print(self._joint_command)
                print(self._velocity_command)
                """
                #field bool trajectory_running
                state.joint_position=self.readJointPositions()
                state.joint_velocity=self.readJointVelocities()
                state.joint_effort=self.readJointTorques()
                state.position_command=self._joint_command
                state.velocity_command=self._velocity_command
                #state.trajectory_running=self._trajectory_running
                """
                state.joint_position=numpy.zeros((0,))
                state.joint_velocity=numpy.zeros((0,))
                state.joint_effort=numpy.zeros((0,))
                state.position_command=numpy.zeros((0,))
                state.velocity_command=numpy.zeros((0,))
                state.trajectory_running=1"""
                self.easy_robot_state.OutValue=state
                self.seqno+=1
                #while (time.time() - t1 < 0.01):
                # idle
            #self._easy_robot_state_sensor_data_broadcaster.AsyncSendPacket(statesensordata,lambda: None)
            time.sleep(self._downsample)
            
    @property
    def easy_robot_state_sensor_data(self):
        return self._robot_state_sensor_data
        
    @easy_robot_state_sensor_data.setter
    def easy_robot_state_sensor_data(self,value):
        self._robot_state_sensor_data=value
        #Create the PipeBroadcaster and set backlog to 3 so packets
        #will be dropped if the transport is overloaded
        self._easy_robot_state_sensor_data_broadcaster=RR.PipeBroadcaster(value,3)
            
    def _robot_header_create(self):
        #header=RRN.NewStructure("com.robotraconteur.sensordata.SensorDataHeader")
        #statesensordata=RRN.NewStructure("com.robotraconteur.sensordata.SensorDataSourceInfo")
        return header
    
    @property
    def EasyRobotInfo(self):
        info=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotInfo")
        
    @property
    def easy_mode(self):
        return self._mode

    @easy_mode.setter
    def easy_mode(self,value):
        self._mode=value
    # worker function to request and update end effector data for sawyer
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        while self._running:
            t1 = time.time()
            self.readEndEffectorPoses()
            self.readEndEffectorTwists()
            self.readEndEffectorWrenches()
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)
            
    # worker function to continuously issue commands to sawyer
    # Try to maintain 100 Hz operation
    # TODO: INCLUDE CLOCK JITTER CORRECTION
    def command_worker(self):
        while self._running:
            t1 = time.time()
            
            if (self._mode == self.MODE_POSITION):
                if(self._jp_pub):
                    self._right.set_joint_positions(self._r_joint_command)
            elif (self._mode == self.MODE_VELOCITY):
                self._right.set_joint_velocities(self._r_joint_command)
            elif (self._mode == self.MODE_TORQUE):
                #self._supp_cuff_int_pubs['left'].publish()
                #self._supp_cuff_int_pubs['right'].publish()
                self._right.set_joint_torques(self._r_joint_command)
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)
        return self._mode
    #@update_downsample.setter
    #def update_downsample(self,downsample):
    #    self._downsample=downsample
    
    @property
    def update_rate(self):
        return self._downsample
        
    @property
    def easy_param_names(self):
        return params
    
    
    
        
        
def main():
    # parse command line arguments
    """parser = argparse.ArgumentParser(
                            description='Initialize Joint Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)"""

    nodename="SawyerJointServer"
    #Initialize object
    sawyer_obj = Sawyer_impl()

    #Create transport, register it, and start the server
    print("Registering Transport")
    
    #port = args.port
    #if (port == 0):
    port =8884 #t.GetListenPort()

    #Register the service type and the service
    print("Starting Service")
    
    #RR.RobotRaconteurNode.s.RegisterServiceTypeFromFile("com.robotraconteur.robotics.easy")
    
    with RR.ServerNodeSetup(nodename,port):
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.geometry")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.uuid")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.datetime")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.sensordata")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.device")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.easy")
        RRN.RegisterService("EasyRobot",
                          "com.robotraconteur.robotics.easy.EasyRobot",
                                              sawyer_obj)
        time.sleep(2)
        sawyer_obj.start()

        print("Service started, connect via")
        print("tcp://localhost:" + str(port) + "/SawyerJointServer/Sawyer")
        raw_input("press enter to quit...\r\n")

        sawyer_obj.close()
    
    # This must be here to prevent segfault
        #RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main()
