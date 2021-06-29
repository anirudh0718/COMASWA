import numpy as np
from robot import Robot_Sim
from plotrob import plot_step
import  matplotlib.pyplot as plt
from FC_to_Goal import get_rob_gposes,get_pose,get_rob_rec_pos,get_turned_rec,get_turn_orient

# Starting postion of our robots
start = []
#start.append(get_rob_gposes(np.array([0,0])))
start.append(get_rob_rec_pos(np.array([0,0])))


# Goal positions of our robots
goal = []
#goal.append(get_rob_gposes(np.array([8,0]))) # sqaure
goal.append(get_turn_orient(np.array([20,20]))) # Turned Rectangle


#print('These are the starting postions of the robots',start)
#print('These are goal position f the robots',goal)


# Inititate our robots

#Robot 1
x_init1 = start[0][0]
goal_init1 =goal[0][0]
robot1 = Robot_Sim(x_init1, goal_init1, 0)

### Robot 2

x_init2 = start[0][1]
goal_init2 =goal[0][1]
Robot2 = Robot_Sim(x_init2, goal_init2,1)

### Robot 3

x_init3 = start[0][2]
goal_init3 =goal[0][2]
Robot3 = Robot_Sim(x_init3, goal_init3,2)

### Robot 4

x_init4 = start[0][3]
goal_init4 =goal[0][3]
Robot4 = Robot_Sim(x_init4, goal_init4,3)

const_obs = np.array([[8], [8]])
const_obs2 = np.array([[5], [5]])

# These are static obstacles we present to the robot
obs = np.hstack((const_obs2, const_obs))

# Add all thge robots to list robots
Robots =[robot1,Robot2,Robot3,Robot4]
N = len(Robots)
poses = []
poses = get_pose(Robots)

a, ax1 = plt.subplots()

for tt in range(1000):

    dxi = np.zeros((2, N))
    safe_dxi = np.zeros((2,N))
    for i in range(N):
        """ IF Id = 1 --> Only obstacle avoidance no formation control
            IF Id = 2 --> Only Formation control 
            IF Id = 3 --> Both obstacle avoidance and Formation control 
            IF Id = 4 --> Only Formation greater case and obstacle avoidance
            IF Id = 5 --> Only Formation lesser case and obstacle avoidance
            Formation greater case = |xi -xj| - (Df - e) > 0"""
        if tt>0:            
            Robots[i].robot_step(obs,Robots,i,3)
            

    if tt%50 ==0:
        print(tt)
        plt.cla()

        for robot in Robots:
            plot_step(robot.state_hist,ax1,obs)
            
        plt.pause(0.1)
for robot in Robots:
    
    robot.ecbf.new_plt_h()
    robot.ecbf.dist_plot()
    