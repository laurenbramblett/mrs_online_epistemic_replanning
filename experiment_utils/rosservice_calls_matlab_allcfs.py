"""Takeoff-hover-land for multiple CF. Useful to validate hardware config."""

import sys
sys.path.insert(1, "~/catkin_ws/src/crazyswarm/crazyswarm/scripts")

from pycrazyswarm import Crazyswarm
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

YAML_FILE =  "~/catkin_ws/src/crazyswarm/crazyswarm/launch/crazyflies_mcts.yaml"

takeoff_height, takeoff_duration = None, None
land_height, land_duration       = None, None

single_land_robot    = None
single_land_height   = None
single_land_duration = None

pos, ori, hover_heights = None, None, None
goTo_coords   = None
worldVel      = None
num_agents    = None

initialized = False
pos_subs    = []
landed_robots = []

def takeoff_cb(data):
  global takeoff_height, takeoff_duration, num_agents

  if data.data != None:
    print(data.data)
    print(num_agents)
    takeoff_height = np.array([data.data[i] for i in range(num_agents)])
    print(takeoff_height)
    takeoff_duration = data.data[num_agents]
    print(takeoff_duration)

def land_cb(data):
  global land_height, land_duration, num_agents
  if data.data != None:
    land_height   = np.array([data.data[i] for i in range(num_agents)])
    land_duration = data.data[num_agents]

def single_land_cb(data):
  global single_land_robot, single_land_height, single_land_duration
  if (data.data != None) and (len(data.data) >= 3) and (data.data[0] < num_agents):
    single_land_robot    = int(data.data[0])
    single_land_height   = data.data[1]
    single_land_duration = data.data[2]
    print(f"Land Single: Robot {single_land_robot}")
  else:
    print("Failure: Single Robot Land")

def goto_cb(data):
	global goTo_coords
	goTo_coords = np.array(data.data)

def worldVel_cb(data):
  global worldVel, num_agents
  #print(np.array(data.data))
  worldVel = np.reshape(np.array(data.data),(num_agents,3))
  #print(worldVel)

def pos_cb(idx, data):
  global pos
  pos[idx,:] = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
  w = data.transform.rotation.w
  z = data.transform.rotation.z
  ori[idx] = math.atan2(2*w*z, math.pow(w,2) - math.pow(z,2))

def callback_factory(k):
    return lambda msg : pos_cb(k, msg)

def robot_list_cb(data):
  global initialized
  global pos_subs, num_agents, landed_robots
  global pos, ori, hover_heights

  rbt_list   = data.data.split(",")
  num_agents = len(rbt_list)
  
  pos = np.zeros((num_agents,3))
  ori = np.zeros((num_agents,1))
  hover_heights = np.zeros((num_agents,1))
  landed_robots = [True]*num_agents
  
  for k in range(num_agents):
    topic   = f"/vicon/{rbt_list[k]}/{rbt_list[k]}"
    pos_subs.append(rospy.Subscriber(topic, TransformStamped, callback_factory(k)))

  print(f"Robots Listed: {rbt_list}")
  print(f"Num Agents {num_agents}")
  initialized = True

def main():
  global takeoff_height, land_height, goTo_coords, worldVel, pos, ori
  global single_land_robot, single_land_height, single_land_duration
  global initialized, pos_subs
  global num_agents

  #rospy.init_node('matlab_msgs',anonymous=True)

  tfsub         = rospy.Subscriber("/takeoff_msg", Float32MultiArray, takeoff_cb)
  landsub       = rospy.Subscriber("/land_msg", Float32MultiArray, land_cb)
  gotosub       = rospy.Subscriber("/goto_msg", Float32MultiArray, goto_cb)
  worldVelsub   = rospy.Subscriber("/vel_msg", Float32MultiArray, worldVel_cb)
  rbtListSub    = rospy.Subscriber("/robot_list", String, robot_list_cb)
  landSingleSub = rospy.Subscriber("/single_rbt_land", Float32MultiArray, single_land_cb)
	
  swarm      = Crazyswarm(crazyflies_yaml=YAML_FILE)
  timeHelper = swarm.timeHelper
  cfs        = swarm.allcfs.crazyflies
  rate       = rospy.Rate(200)

  print("Beginning ROS Loop...")
  while not rospy.is_shutdown():
    
    if initialized:
      if np.all(takeoff_height != None):
        agent_count = 0
        for cf in cfs:
          cf.takeoff(targetHeight=takeoff_height[agent_count], duration=takeoff_duration)
          hover_heights[agent_count] = takeoff_height[agent_count]
          landed_robots[agent_count] = False
          agent_count+=1
        takeoff_height = None

      if np.all(land_height != None):
        agent_count = 0
        for cf in cfs:
          if not landed_robots[agent_count]:
            cf.land(targetHeight=land_height[agent_count], duration=land_duration)
            landed_robots[agent_count] = True
          agent_count+=1
        land_height = None
      
      if (single_land_robot != None) and (single_land_duration != None) and (single_land_height != None):
        if not landed_robots[single_land_robot]:
          cfs[single_land_robot].land(targetHeight=single_land_height, duration=single_land_duration)
          landed_robots[single_land_robot] = True
          single_land_robot    = None
          single_land_duration = None
          single_land_height   = None

      if np.all(worldVel != None):
        agent_count = 0
        initPos = np.array(pos)
        for cf in cfs:
          # print(worldVel)
          if not landed_robots[agent_count]:
            newPos    = initPos[agent_count,:] + worldVel[agent_count,:]
            newPos[2] = hover_heights[agent_count]
            cf.goTo(newPos, 0, 2)
            print(f"{agent_count}: {newPos[0]}, {newPos[1]}, {newPos[2]}")
          agent_count+=1
        worldVel = None
      
    rate.sleep()

if __name__ == "__main__":
	main()
