#!/usr/bin/env python
import roslib
roslib.load_manifest('sawyer_rr_bridge')
import rospy
import intera_interface
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from std_msgs.msg import Empty
from intera_core_msgs.msg import SEAJointState

import sys, argparse
import struct
import time
from collections import OrderedDict
import RobotRaconteur as RR
import thread
import threading
import numpy

sawyer_servicedef="""
#Service to provide simple interface to Sawyer
service SawyerPeripheral_interface

option version 0.4

object SawyerPeripherals

function single getAccelerometerValue(string arm)

function void panHead(double angle)
function single getHeadPanAngle()
function void nodHead()

function void suppressCollisionAvoidance(string limb, uint8 suppress)
function void suppressContactSafety(string limb, uint8 suppress)
function void suppressCuffInteraction(string limb, uint8 suppress)
function void suppressGravityCompensation(string limb, uint8 suppress)

property double[] gravity_compensation_torques

end object

"""
class SawyerPeripherals_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('Sawyer_peripherals')
        
        self._running = True
        self._valid_limb_names = {'right': 'right',
                                    'r': 'right'}
        
        # gripper initialization
        self._grippers = { 'right': intera_interface.Gripper('right')}
        # Set grippers to defaults
        #self._grippers['right'].set_parameters(
        #                        self._grippers['right'].valid_parameters())
                            
        # accelerometer initialization
        self._accelerometers = {'right': [0.0]*3}
        rospy.Subscriber("/robot/accelerometer/right_accelerometer/state", 
                                                                      Imu, 
                                                self.right_accel_callback)
        
        # head control initialization
        self._head = intera_interface.Head()
        
        
        # suppressions                                        
        self._suppress_collision_avoidance = {'right': False}
        self._supp_coll_avoid_pubs = {'right': 
            rospy.Publisher("/robot/limb/right/suppress_collision_avoidance", 
                                                                       Empty, 
                                                                 latch=True)}
                                        
        self._suppress_contact_safety = {'right': False}
        self._supp_con_safety_pubs = {'right': 
            rospy.Publisher("/robot/limb/right/suppress_contact_safety", 
                                                                  Empty, 
                                                            latch=True)}
                                        
        self._suppress_cuff_interaction = {'right': False}
        self._supp_cuff_int_pubs = {'right': 
            rospy.Publisher("/robot/limb/right/suppress_cuff_interaction", 
                                                                    Empty, 
                                                              latch=True)}
                                        
        self._suppress_gravity_compensation = {'right': False}
        self._supp_grav_comp_pubs = {'right': 
            rospy.Publisher("/robot/limb/right/suppress_gravity_compensation", 
                                                                        Empty, 
                                                                  latch=True)}
        
        # start suppressions background thread
        self._t_suppressions = threading.Thread(
                                    target=self.suppressions_worker)
        self._t_suppressions.daemon = True
        self._t_suppressions.start()
        
        # gravity compensation subscription
        self._grav_comp_lock = threading.Lock()
        self._gravity_compensation_torques = OrderedDict( 
                        zip(intera_interface.Limb('right').joint_names(), 
                                                                   [0.0]*7))
        rospy.Subscriber("/robot/limb/right/gravity_compensation_torques", 
                        SEAJointState, self.grav_comp_callback)
   
    def close(self):
        self._running = False
        self._t_suppressions.join()
    
    # Accelerometers
    def getAccelerometerValue(self, arm):
        arm = arm.lower()
        if arm in self._valid_limb_names.keys():
            return self._accelerometers[self._valid_limb_names[arm]]

    def right_accel_callback(self, data):
        if (data.linear_acceleration):
            self._accelerometers['right'][0] = data.linear_acceleration.x
            self._accelerometers['right'][1] = data.linear_acceleration.y
            self._accelerometers['right'][2] = data.linear_acceleration.z
        
    
    # head control functions
    def panHead(self, angle):
        self._head.set_pan(angle)
    
    def getHeadPanAngle(self):
        return self._head.pan()
    
    # Suppression functions
    def suppressCollisionAvoidance(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_collision_avoidance[ \
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_collision_avoidance[ \
                            self._valid_limb_names[limb]] = (suppress > 0)
            if self._suppress_collision_avoidance[ \
                                            self._valid_limb_names[limb]]:
                print 'Suppressing Collision Avoidance for limb ', limb
            else:
                print 'Enabling Collision Avoidance for limb ', limb
    
    def suppressContactSafety(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_contact_safety[ \
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_contact_safety[ \
                            self._valid_limb_names[limb]] = (suppress > 0)
            if self._suppress_contact_safety[self._valid_limb_names[limb]]:
                print 'Suppressing Contact Safety for limb ', limb
            else:
                print 'Enabling Contact Safety for limb ', limb
    
    def suppressCuffInteraction(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_cuff_interaction[\
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_cuff_interaction[self._valid_limb_names[limb]] = \
                                                                (suppress > 0)
            if self._suppress_cuff_interaction[self._valid_limb_names[limb]]:
                print 'Suppressing Cuff Interaction for limb ', limb
            else:
                print 'Enabling Cuff Interaction for limb ', limb
    
    def suppressGravityCompensation(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_gravity_compensation[\
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_gravity_compensation[ \
                                            self._valid_limb_names[limb]] = \
                                                                (suppress > 0)
            if self._suppress_gravity_compensation[ \
                            self._valid_limb_names[limb]]:
                print 'Suppressing Gravity Compensation for limb ', limb
            else:
                print 'Enabling Gravity Compensation for limb ', limb
    
    def publishSuppressions(self, limb):            
        if self._suppress_collision_avoidance[limb]:
            self._supp_coll_avoid_pubs[limb].publish()
        
        if self._suppress_contact_safety[limb]:
            self._supp_con_safety_pubs[limb].publish()
        
        if self._suppress_cuff_interaction[limb]:
            self._supp_cuff_int_pubs[limb].publish()
        
        if self._suppress_gravity_compensation[limb]:
            self._supp_grav_comp_pubs[limb].publish()
    
    # worker function to continuously publish suppression commands at >5Hz
    def suppressions_worker(self):
        while self._running:
            time.sleep(0.05)
            self.publishSuppressions('right')
    
    # gravity compensation info functions
    @property
    def gravity_compensation_torques(self):
        return self._gravity_compensation_torques.values()
    
    def grav_comp_callback(self, data):
        with self._grav_comp_lock:
            if data.gravity_model_effort:
                for n in xrange(0,len(data.name)):
                    self._gravity_compensation_torques[data.name[n]] = \
                                                data.gravity_model_effort[n]           
    

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                        description='Initialize Sawyer Peripherals.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on ' + \
                            '(will auto-generate if not specified)')
    args = parser.parse_args(argv)
    
    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="SawyerPeripheralServer"

    
    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
                         RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
                         RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(sawyer_servicedef)
    
    #Initialize object
    sawyer_obj = SawyerPeripherals_impl()    
    
    RR.RobotRaconteurNode.s.RegisterService("SawyerPeripherals",
                 "SawyerPeripheral_interface.SawyerPeripherals",
                                                     sawyer_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + \
                    "/SawyerPeripheralServer/SawyerPeripherals"
    raw_input("press enter to quit...\r\n")
    
    sawyer_obj.close()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
