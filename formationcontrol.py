import numpy as np
from robot import Robot_Sim
from plotrob import plot_step
import  matplotlib.pyplot as plt
from FC_to_Goal import get_rob_gposes,get_pose,get_rob_rec_pos,get_turned_rec,get_turn_orient


# Define the laplacian and weights for SQUARE AND RECTANGLE
L2 = np.array([
    [2, -1, 0, -1],
    [-1, 3, -1, -1],
    [0, -1, 2, -1],
    [-1, -1 , -1, 3]]) 

# Laplacian matrix for a square/ Rec shape with 4 robots || Constarints for 1 and 3
L = np.array([
    [3, -1, -1, -1],
    [-1, 2, -1, 0],
    [-1, -1, 3, -1],
    [-1, 0 , -1, 2]]) 
# Laplacian matrix for a square/ Rec shape with 4 robots || Constarints for 1 and 3 2 and 4
L3 = np.array([
    [3, -1, -1, -1],
    [-1, 3, -1, -1],
    [-1, -1, 3, -1],
    [-1, -1 , -1, 3]]) 
# Tolerance
e1 = 0.3
e2 = 0.4
# Formation Distance for rectangle shape
Df_l = 3
Df_b = 2

ddiag_rec =np.sqrt(np.power(Df_l,2) + np.power(Df_b,2))  # 2.2360

weights_rec2 = np.array([
    [0, Df_b,0,Df_l],
    [Df_b, 0, Df_l, ddiag_rec],
    [0, Df_l, 0, Df_b],
    [Df_l, ddiag_rec,Df_b, 0]
])

weights_rec = np.array([
    [0, Df_b,ddiag_rec,Df_l],
    [Df_b, 0, Df_l, 0],
    [ddiag_rec, Df_l, 0, Df_b],
    [Df_l, 0,Df_b, 0]
])

weights_rec3 = np.array([
    [0, Df_b,ddiag_rec,Df_l],
    [Df_b, 0, Df_l, ddiag_rec],
    [ddiag_rec, Df_l, 0, Df_b],
    [Df_l, ddiag_rec,Df_b, 0]
])
# Formation Distance for square shape
Df = 2
d_minus_epi = Df

ddiag =np.sqrt(2)*d_minus_epi

# Distance Matrix of one robbot with respect to another
weights = np.array([
    [0, d_minus_epi,ddiag,d_minus_epi],
    [d_minus_epi, 0, d_minus_epi, 0],
    [ddiag, d_minus_epi, 0, d_minus_epi],
    [d_minus_epi, 0,d_minus_epi, 0]
])
# Starting postion of our robots
start = []
#start.append(get_rob_gposes(np.array([0,0])))
start.append(get_rob_rec_pos(np.array([2.5,2.5])))


# Goal positions of our robots
goal = []
#goal.append(get_rob_gposes(np.array([20,20]))) # sqaure
goal.append(get_turn_orient(np.array([25,25]))) # Turned Rectangle


print('These are the starting postions of the robots',start)
print('These are goal position f the robots',goal)


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

const_obs = np.array([[13], [10]])
const_obs2 = np.array([[13], [30]])

# These are static obstacles we present to the robot
obs = np.hstack((const_obs, const_obs2))

#obs = const_obs
# Add all thge robots to list robots
Robots =[robot1,Robot2,Robot3,Robot4]
N = len(Robots)
poses = []
poses = get_pose(Robots)

a, ax1 = plt.subplots()

Top = 1

for tt in range(10000):

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
            #Robots[i].robot_step(obs,Robots,i,3,L,weights_rec)
            #print('Ustar of robot {}'.format(i),np.array(Robots[i].ecbf.compute_safe(obs,Robots,i,3,L,weights_rec)))
            """ if Top == 1 and np.array(Robots[i].ecbf.compute_safe(obs,Robots,i,3,L,weights_rec,e1))[0]<0.1:
                print('SWITCHED TO SECOND TOPOLOGY')
                Top = 2
                # Switch L and weights
                Robots[i].robot_step(obs,Robots,i,3,L2,weights_rec2,e2)
            elif Top ==2 and np.array(Robots[i].ecbf.compute_safe(obs,Robots,i,3,L2,weights_rec2,e2))[0]<0.000001:
                Top =1
                Robots[i].robot_step(obs,Robots,i,3,L,weights_rec,e1)
            else:
                Robots[i].robot_step(obs,Robots,i,3,L,weights_rec,e1) """

            Robots[i].robot_step(obs,Robots,i,3,L3,weights_rec3,e1)
            

    if tt%50 ==0:
        print(tt)
        plt.cla()

        for robot in Robots:
            plot_step(robot.state_hist,ax1,obs)
            
        plt.pause(0.0000001)
for robot in Robots:
    
    robot.ecbf.new_plt_h()
    robot.ecbf.dist_plot()
    