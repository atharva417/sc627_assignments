import numpy as np
import matplotlib.pyplot as plt
import helper

file = open("/home/atharva/syscon_ws/src/sc627_assignments/assignment_2/output.txt","r")
lines = file.readlines()

path = []
for line in lines:
    lin = line.split(',')
    lin = np.float32(lin)
    path.append(np.array([lin[0],lin[1]]))
file.close()
n = len(path)
path = np.array(path)
goal = path[-1]
# print(goal)
dist2g = [helper.distp2p(goal,path[i]) for i in range(n)]

def plot_path():
    plt.title("Path traced by the Bot")
    plt.xlabel("X axis")
    plt.ylabel("Y axis")
    plt.plot(path[:,0], path[:,1], color ="red")
    plt.grid(True)
    plt.xlim(-0.2,5.2)
    plt.ylim(-0.2,4.2)
    plt.savefig("/home/atharva/syscon_ws/src/sc627_assignments/assignment_2/traversed_path.png")
    plt.show()

def plot_distogoal():
    plt.title("Distance from Goal")
    plt.xlabel("Time")
    plt.ylabel("Distance")
    plt.plot(range(n), dist2g, color ="red")
    plt.grid(True)
    plt.savefig("/home/atharva/syscon_ws/src/sc627_assignments/assignment_2/distance_to_goal.png")
    plt.show()

def total_path_length():
    length = 0.0
    for i in range(1,n):
        length+=helper.distp2p(path[i],path[i-1])
    print("Total path length = ",length)

if __name__ == '__main__':
    plot_path()
    # plot_distogoal()
    # total_path_length()