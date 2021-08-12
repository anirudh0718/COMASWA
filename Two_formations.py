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

# Laplacian matrix for a square/ Rec shape with 4 robots || Constarints for 1 and 3 2 and 4 <---
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
e1 = 0.1
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

def get_pose(robots):
    '''
    Returns the position of robots
    '''
    x = np.zeros((2, len(robots)))
    for i in range(len(robots)):
        x[:,i] = robots[i].state['q'][:2].reshape(2,)
    return x


def get_form_cent(rbts):
    pose = get_pose(rbts)
    cent = np.mean(pose,axis=1)
    return cent

def get_angle(poses):
    
    #print(poses[:,0][1])
    diff = poses[:,0] - poses[:,3]
    angle = np.arctan2(diff[1],diff[0])
    return angle


        
# Starting postion of our robots
start = []
#start.append(get_rob_gposes(np.array([0,0])))
start.append(get_rob_rec_pos(np.array([2.5,2.5])))
start.append((get_rob_rec_pos(np.array([13,10]))))

# Goal positions of our robots
goal = []
#goal.append(get_rob_gposes(np.array([20,20]))) # sqaure
goal.append(get_turn_orient(np.array([30,30]))) # Turned Rectangle
goal.append(get_turn_orient(np.array([0,11])))

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

const_obs = np.array([[13], [10]])
const_obs2 = np.array([[13], [22]])



#Formation 2

#Robot 1
x_init21 = start[1][0]
goal_init21 =goal[1][0]
robot21 = Robot_Sim(x_init21, goal_init21, 4)

### Robot 2

x_init22 = start[1][1]
goal_init22 =goal[1][1]
Robot22 = Robot_Sim(x_init22, goal_init22,5)

### Robot 3

x_init23 = start[1][2]
goal_init23 =goal[1][2]
Robot23 = Robot_Sim(x_init23, goal_init23,6)

### Robot 4

x_init24 = start[1][3]
goal_init24 =goal[1][3]
Robot24 = Robot_Sim(x_init24, goal_init24,7)

""" const_obs = np.array([[13], [10]])
const_obs2 = np.array([[13], [22]]) """

const_obs = np.array([[25], [35]])
const_obs2 = np.array([[25], [35]])

form = np.array([[60], [60]])

cent = {'cent_F1':[],'cent_F2':[],'a':3,'b':2,'AF1':0,'AF2':0}

# These are static obstacles we present to the robot
obs = np.hstack((const_obs, const_obs2))

""" print(obs[0])
exit() """

#obs = const_obs
# Add all thge robots to list robots
Robots1 =[robot1,Robot2,Robot3,Robot4]
Robots2 = [robot21,Robot22,Robot23,Robot24]
get_angle(get_pose(Robots1))

roro = [robot1,Robot2,Robot3,Robot4,robot21,Robot22,Robot23,Robot24]
N = len(roro)
""" poses = []
poses = get_pose(Robots1) """

cent['cent_F1'] = get_form_cent(Robots1).reshape(2,1)
cent['cent_F2'] = get_form_cent(Robots2).reshape(2,1)

""" print(cent['cent_F1'])
exit() """

a, ax1 = plt.subplots()

Top = 1

def check_goal_reached(Robots):
    for i in range(N):
        #print(Robots[i].state['q'].shape[0])
        if np.allclose(Robots[i].state['q'],Robots[i].goal,rtol=0, atol=1e-07):
            return True
        else:
            return False



def f_control(N,rbts):

    tt = 0
    while not check_goal_reached(rbts) and tt<10000:

        tt = tt +1

        dxi = np.zeros((2, N))
        safe_dxi = np.zeros((2,N))
        n = 0
        r = 0
        for i in range(4):

            cent['cent_F1'] = get_form_cent(Robots1)
            cent['cent_F2'] = get_form_cent(Robots2) #np.array([10,10]).reshape(2,)

            #print(cent['cent_F1'],cent['cent_F2'])
            cent['AF1'] = get_angle(get_pose(Robots1))
            cent['AF2'] = get_angle(get_pose(Robots2))

            #print('centroid, angle of FORM1 and FORM2: ',cent['cent_F1'],cent['AF1'],cent['cent_F2'],cent['AF2'])
            
            

            
            """ IF Id = 1 --> Only obstacle avoidance no formation control
            IF Id = 2 --> Only Formation control 
            IF Id = 3 --> Both obstacle avoidance and Formation control 
            IF Id = 4 --> Only Formation greater case and obstacle avoidance
            IF Id = 5 --> Only Formation lesser case and obstacle avoidance
            IF Id = 6 --> Obs,ellipse,Form
            IF Id = 7 --> obs,Form,Ellipse
            IF Id = 8 --> Only ellipticalobstacle avoidance
            Formation greater case = |xi -xj| - (Df - e) > 0"""
            if tt>0:
                Robots1[i].robot_step(obs,Robots1,i,i,7,L3,weights_rec3,e1,cent['cent_F2'],cent['AF2'])
                Robots2[i].robot_step(obs,Robots2,i,i,7,L3,weights_rec3,e1,cent['cent_F1'],cent['AF1'])
                """ if tt < 300:
                    #Robots2[i].robot_step(obs,Robots2,i,i,7,L3,weights_rec3,e1,cent['cent_F1'],cent['AF1'])
                    Robots1[i].robot_step(obs,Robots1,i,i,7,L3,weights_rec3,e1,cent['cent_F2'],cent['AF2']) """


                
                

                
          

        if tt%50 ==0:
            print(tt)
            plt.cla()

            for robot in rbts:
                plot_step(robot.state_hist,ax1,obs,robot.n_fcentre,robot.angle)
            
            plt.pause(0.00000001)
    for robot in rbts:
    
        robot.ecbf.new_plt_h(1)
        #robot.ecbf.dist_plot()
    
if __name__ =="__main__":
    print(check_goal_reached(Robots2))

    f_control(N,roro)
    print(check_goal_reached(Robots2))