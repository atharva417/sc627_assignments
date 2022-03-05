#!/usr/bin/env python
import numpy as np
import time
import math
import helper
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
start_time = time.time()

rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
file = open("/home/atharva/syscon_ws/src/sc627_assignments/assignment_1/input.txt","r")
lines = file.readlines()
l1 = lines[0].split(',')
l1 = np.float32(l1)
start = np.array([l1[0],l1[1]])
l2 = lines[1].split(',')
l2 = np.float32(l2)
goal = np.array([l2[0],l2[1]])
step = np.float32(lines[2])


obs = []
obsi = []
for line in lines[4:]:
    if line=="\n":
        obsi = np.array(obsi)
        obs.append(obsi)
        obsi = []
        # print("empty")
    else:
        lin = line.split(',')
        lin = np.float32(lin)
        obsi.append(np.array([lin[0],lin[1]]))
obsi = np.array(obsi)
obs.append(obsi)
file.close()

#setting result as initial location
result = MoveXYResult()
result.pose_final.x = start[0]
result.pose_final.y = start[1]
result.pose_final.theta = 0 #in radians (0 to 2pi)
curr_pos = np.array([start[0],start[1]])

outfile = open("/home/atharva/syscon_ws/src/sc627_assignments/assignment_1/output_bugbase.txt", "w")
outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))

while np.sqrt((curr_pos[0]-goal[0])**2+(curr_pos[1]-goal[1])**2)>step: #replace true with termination condition
    #determine waypoint based on your algo
    dist = []
    for i in obs:
        d,_,_,_ = helper.distp2pol(i,curr_pos)
        dist.append(d)
    dist = np.array(dist)
    pol_ind = np.argmin(dist)
    if np.min(dist)<step:
        xu,yu = helper.tanvec(obs[pol_ind],curr_pos)
    else:
        mag = math.sqrt((goal[0]-curr_pos[0])**2+(goal[1]-curr_pos[1])**2)
        xu,yu = (goal[0]-curr_pos[0])/(mag),(goal[1]-curr_pos[1])/mag

    wp = MoveXYGoal()
    wp.pose_dest.x = curr_pos[0]+step*xu
    wp.pose_dest.y = curr_pos[1]+step*yu
    # print(np.arctan2(yu,xu))
    wp.pose_dest.theta = np.arctan2(yu,xu) #theta is the orientation of robot in radians (0 to 2pi)
    
    client.send_goal(wp)
    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()
    curr_pos = np.array([result.pose_final.x,result.pose_final.y])
    #write to output file (replacing the part below)
    outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))
outfile.write("%s,%s \n" % (goal[0], goal[1]))
outfile.close()
end_time = time.time()
total_time = end_time - start_time
print("Execution Time: ", total_time)