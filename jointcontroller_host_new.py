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
from copy import copy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

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
        self._ns = 'robot/limb/right'
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        #self.trajectory_client = actionlib.SimpleActionClient(self._fjt_ns, FollowJointTrajectoryAction)
        
        #self.trajectory_client.wait_for_server()
        self.current_plan=None
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
        self._speed_ratio=0
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
        
        
    def jog_joint(self, joint_positions, max_velocity=[], relative=None, wait=None):
        print("jogging")
        print(joint_positions)
        self.setJointCommand('right',joint_positions)

    #def execute_trajectory(self, joint_positions, joint_velocities)
        

    def jog_cartesian(self, target_pose, max_velocity,relative, wait):
        #target_pose[0]['position']['x']
        end_effector_position=[target_pose[0]['position']['x'],target_pose[0]['position']['y'],target_pose[0]['position']['z']]
        end_effector_quaternion=[target_pose[0]['orientation']['w'],target_pose[0]['orientation']['x'],target_pose[0]['orientation']['y'],target_pose[0]['orientation']['z']]
        #end_effector_position=[target_pose.translation.x,target_pose.translation.y,target_pose.translation.z]
        #end_effector_quaternion=[target_pose.orientation.w,target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z]
        joint_angles=ik_service_client(end_effector_position, end_effector_quaternion)
        self.setJointCommand('right',joint_angles)

    def execute_trajectory(self,trajectory):
        self.trajectory_running=True
        self._current_trajectory=trajectory
        return trajectory_generator(self)

    def jointspace_worker(self):
        while self._running:
            #statesensordata=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotStateSensorData")
            with self._lock:
                state=RRN.NewStructure("com.robotraconteur.robotics.robot.RobotState")
                #mode=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotMode")

                
                #print(self._right.endpoint_pose()['position'])
                #state.seqno=self.seqno
                state.seqno=self.seqno
                
                #print(self._mode)
                state.controller_mode=self._mode
                state.operational_mode=1
                state.controller_state=self._mode
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
                state.trajectory_running=False
                self.readEndEffectorPoses()
                pose=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose")
                position=numpy.zeros((1,),dtype=pose)
                position[0]['orientation']['w']=self._ee_or[0]
                position[0]['orientation']['x']=self._ee_or[1]
                position[0]['orientation']['y']=self._ee_or[2]
                position[0]['orientation']['z']=self._ee_or[3]
                position[0]['position']['x']=self._ee_pos[0]
                position[0]['position']['y']=self._ee_pos[1]
                position[0]['position']['z']=self._ee_pos[2]
                #pose_temp=self._ee_or+self._ee_pos
                #position=numpy.ArrayToNamedArray(pose_temp)
                #position.orientation.w=self._ee_or[0]
                #position.orientation.x=self._ee_or[1]
                #position.orientation.y=self._ee_or[2]
                #position.orientation.z=self._ee_or[3]
                #position.position.x=self._ee_pos[0]
                #position.position.y=self._ee_pos[1]
                #position.position.z=self._ee_pos[2]
                state.kin_chain_tcp=position
                #state.trajectory_running=self._trajectory_running
                
                """
                state.joint_position=numpy.zeros((0,))
                state.joint_velocity=numpy.zeros((0,))
                state.joint_effort=numpy.zeros((0,))
                state.position_command=numpy.zeros((0,))
                state.velocity_command=numpy.zeros((0,))
                state.trajectory_running=1"""
                self.robot_state.OutValue=state
                self.seqno+=1
                #while (time.time() - t1 < 0.01):
                # idle
            #self._easy_robot_state_sensor_data_broadcaster.AsyncSendPacket(statesensordata,lambda: None)
            time.sleep(self._downsample)
    """        
    @property
    def easy_robot_state_sensor_data(self):
        return self._robot_state_sensor_data
        
    @easy_robot_state_sensor_data.setter
    def easy_robot_state_sensor_data(self,value):
        self._robot_state_sensor_data=value
        #Create the PipeBroadcaster and set backlog to 3 so packets
        #will be dropped if the transport is overloaded
        self._easy_robot_state_sensor_data_broadcaster=RR.PipeBroadcaster(value,3)
    """        
    def _robot_header_create(self):
        #header=RRN.NewStructure("com.robotraconteur.sensordata.SensorDataHeader")
        #statesensordata=RRN.NewStructure("com.robotraconteur.sensordata.SensorDataSourceInfo")
        return header
    
    @property
    def EasyRobotInfo(self):
        info=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotInfo")
    
    @property
    def speed_ratio(self):
        
        return self._speed_ratio

    @speed_ratio.setter
    def speed_ratio(self,value):
        print("Changing speed ratio to: "+str(value))
        self._speed_ratio=value

    @property
    def command_mode(self):
        
        return self._mode

    @command_mode.setter
    def command_mode(self,value):
        print("Changing mode to: "+str(value))
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
    
class trajectory_generator(object):
    def __init__(self,robot_object):
        self._j=0
        self._closed=False
        self.robot_object=robot_object
        self._aborted=False
        self.duration_from_start=0
        #self._goal = FollowJointTrajectoryGoal()
        #joint_names=[]
        #for i in self.robot_object._current_trajectory.joint_names:
        #    joint_names.append(str(i))
        #self._goal.trajectory.joint_names=joint_names
        #self._goal_time_tolerance = rospy.Time(0.1)
        #self._goal.goal_time_tolerance = self._goal_time_tolerance
        

    def Next(self): #add joints next
        trajectory_status=RRN.NewStructure("com.robotraconteur.robotics.trajectory.TrajectoryStatus")
        if self._aborted:
            self.robot_object.trajectory_running=False
            self.robot_object._current_trajectory=None
            trajectory_status.status= -1
            raise OperationAbortedException()
        #check if number of items = joint number and error
        elif self._closed:
            
            self.robot_object.trajectory_running=False
            self.robot_object._current_trajectory=None
            trajectory_status.status=3
            raise StopIterationException()
        elif self._j>=(len(self.robot_object._current_trajectory.waypoints)):
            trajectory_status.status=3
            #self._goal.trajectory.header.stamp = rospy.Time.now()
            #print(self._goal)
            #result=self.robot_object.trajectory_client.send_goal(self._goal)
            
            #self.robot_object.trajectory_client.wait_for_result(rospy.Duration(30.0))
            #print(self.robot_object.trajectory_client.get_result())
            trajectory_status.seqno=self.robot_object.seqno
            trajectory_status.current_waypoint=self._j
            self.duration_from_start=(self.robot_object._speed_ratio)*self._j
            trajectory_status.trajectory_time=self.duration_from_start
            print("sending and finishing")
            self._j=0
            raise StopIterationException()
        else:
            trajectory_status.status=2
            print("continuing")
        waypoint=self.robot_object._current_trajectory.waypoints[self._j]
        print(self._j)
        print(waypoint.joint_position)

        #print(len(self.robot_object._current_trajectory.waypoints)-1)
        #point = JointTrajectoryPoint()
        #point.positions = list(waypoint.joint_position)
        #point.time_from_start = rospy.Duration(waypoint.time_from_start)
        
        #self._goal.trajectory.points.append(point)
        self.robot_object.jog_joint(waypoint.joint_position,waypoint.joint_velocity)
        time.sleep(1)
        #if (self._j>=8):
        #    raise StopIterationException()
        
        #a = copy.copy(v)
        #for i in xrange(len(a)):
        #    a[i]+=self._j
        
        trajectory_status.seqno=self.robot_object.seqno
        trajectory_status.current_waypoint=self._j
        self.duration_from_start=(self.robot_object._speed_ratio)*self._j
        trajectory_status.trajectory_time=self.duration_from_start
        
        self._j+=1
        
        return trajectory_status
        
    def Abort(self):
        self._aborted=True
        
    def Close(self):
        self._closed=True
        
        

    
def ik_service_client(position_in,orientation_in):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position_in[0],
                    y=position_in[1],
                    z=position_in[2],
                ),
                orientation=Quaternion(
                    x=orientation_in[0],
                    y=orientation_in[1],
                    z=orientation_in[2],
                    w=orientation_in[3],
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses["right"])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)
        output=[limb_joints['right_j0'],limb_joints['right_j1'],limb_joints['right_j2'],limb_joints['right_j3'],limb_joints['right_j4'],limb_joints['right_j5'],limb_joints['right_j6']]
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False
    print output
    return output
        
        
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
	RRN.RegisterServiceTypeFromFile("com.robotraconteur.identifier")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.sensordata")
	RRN.RegisterServiceTypeFromFile("com.robotraconteur.resource")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.device")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.units")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.joints")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.trajectory")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.datatype")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.signal")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.param")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.tool")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.payload")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.robot")
        RRN.RegisterService("Sawyer",
                          "com.robotraconteur.robotics.robot.Robot",
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
