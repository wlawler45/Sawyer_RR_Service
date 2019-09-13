#!/usr/bin/env python
import roslib
roslib.load_manifest('sawyer_rr_bridge')
import rospy
import intera_interface
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback

sawyer_servicedef="""
#Service to provide simple interface to Sawyer
service SawyerCamera_interface

option version 0.4

struct SawyerImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
    field string encoding
    field int32 is_bigendian
end struct

struct ArMarker
    field string markerdata
end struct

struct CameraIntrinsics
    field double[] K
    field double[] D
end struct

struct ImageHeader
end struct

object SawyerCamera

    property uint8 camera_open

    # camera control functions
    function void openCamera()
    function void closeCamera()
    function void openLight()
    function void closeLight()
    
    
    # functions to acquire data on the image
    function SawyerImage getCurrentImage()
    function ImageHeader getImageHeader()
    function CameraIntrinsics getCameraIntrinsics()
    
    # pipe to stream images through
    pipe SawyerImage ImageStream
    
end object

"""
class SawyerCamera_impl(object):
    def __init__(self, camera_name):#, mode, half_res):
        rp = intera_interface.RobotParams()
        valid_cameras = rp.get_camera_names()
        if not valid_cameras:
            rp.log_message(("Cannot detect any camera_config"
                " parameters on this robot. Exiting."), "ERROR")
            return
            
        print "Initializing ROS Node"
        rospy.init_node('sawyer_cameras', anonymous=True)
          
        
        # Lock for multithreading
        self._lock = threading.RLock()
        
        # for image pipe
        self._imagestream=None
        self._imagestream_endpoints = dict()
        self._imagestream_endpoints_lock = threading.RLock()
        
        # get access to camera controls from Intera SDK
        self._camera = intera_interface.Cameras()
        if not self._camera.verify_camera_exists(camera_name):
            rospy.logerr("Invalid camera name, exiting the example.")
            return

        self._camera_name = camera_name;
        
        # automatically close camera at start
        self._camera.stop_streaming(self._camera_name);
        self._camera_open = False
        self._ar_tracking = False
        
        # set constant ImageHeader structure
        self._image_header = RR.RobotRaconteurNode.s.NewStructure( 
                                    "SawyerCamera_interface.ImageHeader" )
        
        
        self._camera_intrinsics = None
        
        # set SawyerImage struct
        self._image = RR.RobotRaconteurNode.s.NewStructure("SawyerCamera_interface.SawyerImage")

        # get access to light controls from Intera SDK
        self._lights = intera_interface.Lights()


    # open light
    def openLight(self):
        self._lights.set_light_state('right_hand_blue_light',True)

    # close light
    def closeLight(self):
        self._lights.set_light_state('right_hand_blue_light',False)
        
    # open camera 
    def openCamera(self):
        if self._camera_open:
            return
        
        # start camera and subscription
        self._camera.start_streaming(self._camera_name)
        
        # get camera intrinsic values and fill Robot Raconteur struct
        self._caminfo_sub = rospy.Subscriber("io/internal_camera/" + self._camera_name + "/camera_info", 
                                CameraInfo, self.set_CameraIntrinsics)
        # Suscriber to camera image
        print "Subscribing to", self._camera_name
        self._image_sub = rospy.Subscriber("io/internal_camera/" + self._camera_name + "/image_rect", Image, self.set_imagedata) 
        self._camera_open = True

    def stopArTracking(self):
        if not self._ar_tracking:
            return
	
	if self._marker_sub:
            self._marker_sub.unregister()
       
        self._ar_tracking = False
    
    
    def closeCamera(self):
        if not self._camera_open:
            return
    
        if self._image_sub:
            self._image_sub.unregister()

        self._camera.stop_streaming(self._camera_name)
        self._camera_open = False
    
    @property
    def camera_open(self):
        if self._camera_open:
            return 1
        else:
            return 0

    def set_markerdata(self,markerdata):
        with self._lock:
            if markerdata:
		self._marker= markerdata  
    def getCurrentMarker(self):
        with self._lock:
            return self._marker      
        
    
    # subscriber function for camera image
    def set_imagedata(self,camdata):
        with self._lock:
            if camdata.width and camdata.height and camdata.step and camdata.data:
		self._image.width = camdata.width
		self._image.height = camdata.height
		self._image.step = camdata.step
                self._image.data = numpy.frombuffer(camdata.data,dtype="u1")
		self._image.encoding = camdata.encoding
		self._image.is_bigendian = camdata.is_bigendian

        
        with self._imagestream_endpoints_lock:
            # Send to pipe endpoints
            for ep in self._imagestream_endpoints:
                dict_ep = self._imagestream_endpoints[ep]
                # Loop through indices in nested dict
                for ind in dict_ep:
                    # Attempt to send frame to connected endpoint
                    try:
                        pipe_ep=dict_ep[ind]
                        pipe_ep.SendPacket(self._image)
                    except:
                        # on error, assume pipe has been closed
                        self.ImageStream_pipeclosed(pipe_ep)
                        
    
    def set_CameraIntrinsics(self,data):
        if (self._camera_intrinsics is None):
            print "Setting Camera Intrinsic Data"
            self._camera_intrinsics = RR.RobotRaconteurNode.s.NewStructure( 
                                    "SawyerCamera_interface.CameraIntrinsics" )
            K = list(data.K)
            K[2] -= data.roi.x_offset;
            K[5] -= data.roi.y_offset;
            self._camera_intrinsics.K = tuple(K)
            self._camera_intrinsics.D = tuple(data.D)
            self._caminfo_sub.unregister()
    
    def getCurrentImage(self):
        with self._lock:
            return self._image
    
    def getImageHeader(self):
        return self._image_header
    
    def getCameraIntrinsics(self):
        return self._camera_intrinsics
    
    # pipe functions
    @property
    def ImageStream(self):
        return self._imagestream
    
    @ImageStream.setter
    def ImageStream(self, value):
        self._imagestream = value
        # Set the PipeConnecCallback to ImageStream_pipeconnect that will
        # called when a PipeEndpoint connects
        value.PipeConnectCallback = self.ImageStream_pipeconnect
    
    def ImageStream_pipeconnect(self, pipe_ep):
        # Lock the _imagestream_endpoints dictionary and place he pipe_ep in 
        # a nested dict that is indexed by the endpoint of the client and the 
        # index of the pipe
        with self._imagestream_endpoints_lock:
            # if there is not an enry for this client endpoint, add it
            if (not pipe_ep.Endpoint in self._imagestream_endpoints):
                self._imagestream_endpoints[pipe_ep.Endpoint] = dict()
            
            # Add pipe_ep to the correct dictionary given the endpoint + index
            dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
            dict_ep[pipe_ep.Index] = pipe_ep
            pipe_ep.PipeEndpointClosedCallback = self.ImageStream_pipeclosed
    
    def ImageStream_pipeclosed(self, pipe_ep):
        with self._imagestream_endpoints_lock:
            try:
                dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
                del(dict_ep[pipe_ep.Index])
            except:
                traceback.print_exc()
    

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(description='Initialize Sawyer Camera.')
    parser.add_argument('camera_name', metavar='camera_name',
			   choices=['right_hand_camera', 'head_camera'], 
			   help='name of the camera to connect to')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on (will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the RobotRaconteur Node name
    RR.RobotRaconteurNode.s.NodeName="SawyerCameraServer"

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
        RR.IPNodeDiscoveryFlags_LINK_LOCAL | RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(sawyer_servicedef)
    
    #Initialize object
    sawyer_obj = SawyerCamera_impl(args.camera_name)
    
    RR.RobotRaconteurNode.s.RegisterService(args.camera_name, 
				"SawyerCamera_interface.SawyerCamera", sawyer_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/SawyerCameraServer/" + args.camera_name
    raw_input("press enter to quit...\r\n")

    sawyer_obj.closeCamera()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
