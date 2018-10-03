#!/usr/bin/env python

# AirSim Python API
import setup_path 
import airsim

import rospy

nodeName = "flightroutine_node"

def initializeNode(name):
	rospy.init_node(name, log_level=rospy.INFO, disable_signals=False)

def waitForStartSignal():
	pass

def flight():
	client = airsim.MultirotorClient()
	client.confirmConnection()
	client.enableApiControl(True)
	client.armDisarm(True)

	rospy.loginfo("Starting Flight. Landed: %d", client.getMultirotorState().landed_state)
	client.takeoffAsync().join()
	rospy.loginfo("Flying. Landed: %d", client.getMultirotorState().landed_state)
	client.moveToPositionAsync(0, 0, 5, 5).join()
	client.hoverAsync().join()
	
	client.moveToPositionAsync(15, 0, 5, 2).join()
	client.moveToPositionAsync(0, 15, 5, 2).join()
	client.moveToPositionAsync(-15, 0, 5, 2).join()
	client.moveToPositionAsync(0, -15, 5, 2).join()
	client.hoverAsync().join()

	rospy.loginfo("Landing")
	client.moveToPositionAsync(0, 0, 25, 5).join()
	while not rospy.is_shutdown() and client.getMultirotorState().landed_state != 0:
		client.landAsync().join()

	rospy.spin()

	rospy.loginfo("Disarming, resetting and releasing API control")
	client.armDisarm(False)
	client.enableApiControl(False)
	rospy.loginfo("Shutting down")

if __name__ == '__main__':
    try:
    	initializeNode(nodeName)
    	waitForStartSignal()
        flight()
    except rospy.ROSInterruptException:
    	pass