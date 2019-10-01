from RobotRaconteur.Client import *
import numpy as np

def main():

    url='rr+tcp://localhost:8884/?service=Sawyer'
    

    #Start up Robot Raconteur and connect, standard by this point    
    c_host=RRN.ConnectService(url)
    c_host.command_mode=3
    trajectory=RRN.NewStructure('com.robotraconteur.robotics.trajectory.JointTrajectory',c_host)
    trajectory.joint_names=["head_pan", "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
    trajectory.joint_units=[0,0,0,0,0,0,0,0]
    waypoint0=RRN.NewStructure('com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint',c_host)
    waypoint0.joint_position=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    waypoint0.joint_velocity=np.zeros(8)
    waypoint0.position_tolerance=np.zeros(8)
    waypoint0.velocity_tolerance=np.zeros(8)
    waypoint0.interpolation_mode=0
    waypoint0.time_from_start=1.0
    waypoint1=RRN.NewStructure('com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint',c_host)
    waypoint1.joint_position=np.array([0.5,1.0,2.0,2.0,2.0,2.0,2.0,0.0])
    waypoint1.joint_velocity=np.zeros(8)
    waypoint1.position_tolerance=np.zeros(8)
    waypoint1.velocity_tolerance=np.zeros(8)
    waypoint1.interpolation_mode=0
    waypoint1.time_from_start=4.0
    
    trajectory.waypoints=[waypoint0,waypoint1]
    """
    struct JointTrajectoryWaypoint
    field double[] joint_position
    field double[] joint_velocity
    field double[] position_tolerance
    field double[] velocity_tolerance
	field InterpolationMode interpolation_mode
    field double time_from_start
    end
    struct JointTrajectory	
	field string{list} joint_names
	field JointPositionUnits{list} joint_units
    field JointTrajectoryWaypoint{list} waypoints
	field varvalue{string} extended
    end"""
    genny=c_host.execute_trajectory(trajectory)
    #genny.Next([])
    out=None
    for i in trajectory.waypoints:
        
        out=genny.Next()
    out=genny.Next()
    #out=genny.Next()
    print(out.current_waypoint)
    

if __name__ == '__main__':
    main()
