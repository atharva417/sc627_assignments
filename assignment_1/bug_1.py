#!/usr/bin/env python
import numpy as np
import math
import time
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

outfile = open("/home/atharva/syscon_ws/src/sc627_assignments/assignment_1/output_bug1.txt", "w")
outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))

while np.sqrt((curr_pos[0]-goal[0])**2+(curr_pos[1]-goal[1])**2)>step:
    dist = []
    for i in obs:
        d,_,_,_ = helper.distp2pol(i,curr_pos)
        dist.append(d)
    dist = np.array(dist)
    pol_ind = np.argmin(dist)

    if np.min(dist)<step:
        min_dis=helper.distp2p(curr_pos,goal)
        min_pt = curr_pos
        return_pt = curr_pos

        xu,yu = helper.tanvec(obs[pol_ind],curr_pos)
        wp = MoveXYGoal()
        wp.pose_dest.x = curr_pos[0]+step*xu
        wp.pose_dest.y = curr_pos[1]+step*yu
        wp.pose_dest.theta = np.arctan2(yu,xu)
        client.send_goal(wp)
        client.wait_for_result()
        result = client.get_result()
        curr_pos = np.array([result.pose_final.x,result.pose_final.y])
        outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))

        while(helper.distp2p(curr_pos,return_pt)>(step/2)):
            # print('in_while')
            if helper.distp2p(curr_pos,goal)<min_dis:
                min_dis=helper.distp2p(curr_pos,goal)
                min_pt = curr_pos

            xu,yu = helper.tanvec(obs[pol_ind],curr_pos)
            wp = MoveXYGoal()
            wp.pose_dest.x = curr_pos[0]+step*xu
            wp.pose_dest.y = curr_pos[1]+step*yu
            wp.pose_dest.theta = np.arctan2(yu,xu)
            client.send_goal(wp)
            client.wait_for_result()
            result = client.get_result()
            curr_pos = np.array([result.pose_final.x,result.pose_final.y])
            outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))
            md_pol,_,_,_ = helper.distp2pol(obs[pol_ind],curr_pos)
            if md_pol>step:
                xu,yu = helper.perpvec(obs[pol_ind],curr_pos)
                wp = MoveXYGoal()
                wp.pose_dest.x = curr_pos[0]+step*xu/2
                wp.pose_dest.y = curr_pos[1]+step*yu/2
                wp.pose_dest.theta = np.arctan2(yu,xu)
                client.send_goal(wp)
                client.wait_for_result()
                result = client.get_result()
                curr_pos = np.array([result.pose_final.x,result.pose_final.y])
                outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))

        while(helper.distp2p(curr_pos,min_pt)>(4*step/5)):
            xu,yu = helper.tanvec(obs[pol_ind],curr_pos)
            wp = MoveXYGoal()
            wp.pose_dest.x = curr_pos[0]+step*xu
            wp.pose_dest.y = curr_pos[1]+step*yu
            wp.pose_dest.theta = np.arctan2(yu,xu)
            client.send_goal(wp)
            client.wait_for_result()
            result = client.get_result()
            curr_pos = np.array([result.pose_final.x,result.pose_final.y])
            outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))
            md_pol,_,_,_ = helper.distp2pol(obs[pol_ind],curr_pos)
            if md_pol>step:
                xu,yu = helper.perpvec(obs[pol_ind],curr_pos)
                wp = MoveXYGoal()
                wp.pose_dest.x = curr_pos[0]+step*xu/2
                wp.pose_dest.y = curr_pos[1]+step*yu/2
                wp.pose_dest.theta = np.arctan2(yu,xu)
                client.send_goal(wp)
                client.wait_for_result()
                result = client.get_result()
                curr_pos = np.array([result.pose_final.x,result.pose_final.y])
                outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))

        mag = math.sqrt((goal[0]-curr_pos[0])**2+(goal[1]-curr_pos[1])**2)
        xu,yu = (goal[0]-curr_pos[0])/(mag),(goal[1]-curr_pos[1])/mag
        wp = MoveXYGoal()
        wp.pose_dest.x = curr_pos[0]+1.2*step*xu
        wp.pose_dest.y = curr_pos[1]+1.2*step*yu
        wp.pose_dest.theta = np.arctan2(yu,xu)
        client.send_goal(wp)
        client.wait_for_result()
        result = client.get_result()
        curr_pos = np.array([result.pose_final.x,result.pose_final.y])
        outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))
        # print('obstacle_done')
    else:
        mag = math.sqrt((goal[0]-curr_pos[0])**2+(goal[1]-curr_pos[1])**2)
        xu,yu = (goal[0]-curr_pos[0])/(mag),(goal[1]-curr_pos[1])/mag
        wp = MoveXYGoal()
        wp.pose_dest.x = curr_pos[0]+step*xu
        wp.pose_dest.y = curr_pos[1]+step*yu
        wp.pose_dest.theta = np.arctan2(yu,xu)
        client.send_goal(wp)
        client.wait_for_result()
        result = client.get_result()
        curr_pos = np.array([result.pose_final.x,result.pose_final.y])
        outfile.write("%s,%s \n" % (curr_pos[0], curr_pos[1]))
outfile.write("%s,%s \n" % (goal[0], goal[1]))
outfile.close()
end_time = time.time()
total_time = end_time - start_time
print("Execution Time: ", total_time)