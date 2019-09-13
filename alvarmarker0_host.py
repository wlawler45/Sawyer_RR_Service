#!/usr/bin/env python
import roslib
roslib.load_manifest('sawyer_rr_bridge')
import rospy
import intera_interface
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
#from geometry_msgs.msg.PoseStamped import pose


import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback

alvar_servicedef="""
#Service to provide simple interface to Alvar
service Alvar_interface

option version 0.4

object Alvar

    property double[] marker_positions
    property double[] marker_orientations

end object

"""
class Alvar_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('alvar_markerstates', anonymous=True)

        # Lock for multithreading
        self._lock = threading.RLock()
        
	self._running = True
        self._mk_pos = [0]*3
        self._mk_or = [0]*4

        self._marker_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self._on_markerdata) 
        
        ##self._t_mk = threading.Thread(target=self.marker_worker)
        ##self._t_mk.daemon = True
        ##self._t_mk.start()

    # subscriber function for camera image
    def _on_markerdata(self,alvardata):
        with self._lock:
	    if alvardata.markers:            
            	self._mk_pos[0] = alvardata.markers[0].pose.pose.position.x
            	self._mk_pos[1] = alvardata.markers[0].pose.pose.position.y
            	self._mk_pos[2] = alvardata.markers[0].pose.pose.position.z
            	self._mk_or[0] = alvardata.markers[0].pose.pose.orientation.x
            	self._mk_or[1] = alvardata.markers[0].pose.pose.orientation.y
            	self._mk_or[2] = alvardata.markers[0].pose.pose.orientation.z
            	self._mk_or[3] = alvardata.markers[0].pose.pose.orientation.w
	    else:
		self._mk_pos = [0]*3
       		self._mk_or = [0]*4
      

    def close(self):
        self._running = False        
        #self._t_marker.join()
        if self._marker_sub:
            self._marker_sub.unregister()

    @property 
    def marker_positions(self):
        return self._mk_pos
    
    @property 
    def marker_orientations(self):
        return self._mk_or
     
    ##def readMarkerPoses(self):
	##r_pose = self._right.endpoint_pose()
        ##if r_pose:
            ##self._mk_pos[0] = r_pose['position'].x
            ##self._mk_pos[1] = r_pose['position'].y
            ##self._mk_pos[2] = r_pose['position'].z
            ##self._mk_or[0] = r_pose['orientation'].w
            ##self._mk_or[1] = r_pose['orientation'].x
            ##self._mk_or[2] = r_pose['orientation'].y
            ##self._mk_or[3] = r_pose['orientation'].z       
    
    # worker function to request and update end effector data for sawyer
    # Try to maintain 100 Hz operation
    ##def endeffector_worker(self):
        ##while self._running:
            ##t1 = time.time()
            ##self.readMarkerPoses()
            ##while (time.time() - t1 < 0.01):
                # idle
                ##time.sleep(0.001)

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(description='Initialize Alvar Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="AlvarServer"

    #Initialize object
    alvar_obj = Alvar_impl()

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
    RR.RobotRaconteurNode.s.RegisterServiceType(alvar_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("Marker0",
                      "Alvar_interface.Alvar", alvar_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/AlvarServer/Marker0"
    raw_input("press enter to quit...\r\n")
    
    alvar_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
