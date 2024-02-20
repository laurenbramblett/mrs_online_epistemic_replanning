"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

takeoff_height   = None
land_height      = None
goTo_coords      = None
takeoff_duration = None
land_duration    = None
worldVel         = None
pos              = None
ori              = None


def pos_cb(data):
	global pos
	pos = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
	w = data.transform.rotation.w
	z = data.transform.rotation.z
	ori = math.atan2(2*w*z, math.pow(w,2) - math.pow(z,2))
	
def takeoff_cb(data):
	global takeoff_height; global takeoff_duration
	if data.data != None:
		takeoff_height   = data.data[0]
		takeoff_duration = data.data[1]
		
def land_cb(data):
	global land_height, land_duration
	if data.data != None:
		land_height   = data.data[0]
		land_duration = data.data[1]
		
def goto_cb(data):
	global goTo_coords
	goTo_coords = np.array(data.data)
	
def worldVel_cb(data):
	global worldVel
	worldVel = np.array(data.data)

def main():
  global takeoff_height, land_height, goTo_coords, worldVel, pos, ori
  #rospy.init_node('matlab_msgs',anonymous=True)

  possub      = rospy.Subscriber("/vicon/cf3/cf3", TransformStamped, pos_cb) #Change for new cf
  tfsub       = rospy.Subscriber("/takeoff_msg", Float32MultiArray, takeoff_cb)
  landsub     = rospy.Subscriber("/land_msg", Float32MultiArray, land_cb)
  gotosub     = rospy.Subscriber("/goto_msg", Float32MultiArray, goto_cb)
  worldVelsub = rospy.Subscriber("/vel_msg", Float32MultiArray, worldVel_cb)

  swarm = Crazyswarm()
  timeHelper = swarm.timeHelper
  cfs  = swarm.allcfs.crazyflies

  rate = rospy.Rate(200)
  rate.sleep()
  while not rospy.is_shutdown():
    #print(pos)

    # Request Crazyflie takeoff?
    if takeoff_height != None:		
      print(takeoff_height)
      agent_count = 0
      for cf in cfs:
        cf.takeoff(targetHeight=takeoff_height[agent_count], duration=takeoff_duration[agent_count])
        agent_count+=1
      #timeHelper.sleep(takeoff_height + takeoff_duration)
		


    if land_height != None:
      print(land_height)
      agent_count = 0
      for cf in cfs:
        # (if land
        cf.land(targetHeight=land_height[agent_count], duration=land_duration[agent_count])
        agent_count+=1
			#timeHelper.sleep(land_duration)

		# if np.all(goTo_coords != None):
		# 	cf.goTo(goTo_coords[:3],0,goTo_coords[3])

		# if np.all(worldVel != None):
		# 	initPos = np.array(pos)
		# 	#initPos[2] = 0.7 #Keep at stable height
		# 	newPos = initPos + worldVel[:3]
		# 	print(newPos)
		# 	cf.goTo(newPos, 0, worldVel[3])
		# takeoff_height, land_height, goTo_coords, worldVel = None,None,None,None
		# rate.sleep()

if __name__ == "__main__":
	main()
