import numpy as np
from robot import Robot_Sim
from plotrob import plot_step
import  matplotlib.pyplot as plt
from FC_to_Goal import get_rob_gposes,get_pose,get_rob_rec_pos,get_turned_rec,get_turn_orient
from formge import gen2
from math import degrees
# Tolerance
e1 = 0.1
e2 = 0.4
# Formation Distance for rectangle shape
Df_l = 3
Df_b = 2

ddiag_rec =np.sqrt(np.power(Df_l,2) + np.power(Df_b,2))  # 2.2360

# Laplacian matrix for a square/ Rec shape with 4 robots || Constarints for 1 and 3 2 and 4
L3 = np.array([
    [3, -1, -1, -1],
    [-1, 3, -1, -1],
    [-1, -1, 3, -1],
    [-1, -1 , -1, 3]]) 

weights_rec3 = np.array([
    [0, Df_b,ddiag_rec,Df_l],
    [Df_b, 0, Df_l, ddiag_rec],
    [ddiag_rec, Df_l, 0, Df_b],
    [Df_l, ddiag_rec,Df_b, 0]
])
L2 = np.array([
    [1, -1],
    [-1, 1]]) 

weights_rec2 = np.array([
    [0, Df_l],
    [Df_l, 0]
])
# Starting postion of our robots
#start = []
#start.append(get_rob_gposes(np.array([0,0])))
#start.append(get_rob_rec_pos(np.array([-3,-2])))
#start.append((get_rob_rec_pos(np.array([25,25]))))
start1,start2 = gen2(np.array([0,0]).reshape(2,))
goal1,goal2 = gen2(np.array([20,0]).reshape(2,))
start3,start4 = gen2(np.array([20,0]).reshape(2,))
goal3,goal4 = gen2(np.array([0,0]).reshape(2,))
# Goal positions of our robots
goal = []
#goal.append(get_rob_gposes(np.array([20,20]))) # sqaure
#goal.append(get_turn_orient(np.array([25,19]))) # Turned Rectangle
#goal.append(get_turn_orient(np.array([3,0])))

#Robot 1
x_init1 = start1
goal_init1 =goal1
robot1 = Robot_Sim(x_init1, goal_init1, 0)

#Robot 2
x_init2 = start2
goal_init2 =goal2
robot2 = Robot_Sim(x_init2, goal_init2, 1)

#Robot 3
x_init3 = start3
goal_init3 =goal3
robot3 = Robot_Sim(x_init3, goal_init3, 2)

#Robot 4
x_init4 = start4
goal_init4 =goal4
robot4 = Robot_Sim(x_init4, goal_init4, 3)

roro = [robot1,robot2]
roro1 = [robot3,robot4]
rbts12 = [robot1,robot2,robot3,robot4]
const_obs = np.array([[30], [40]])
const_obs2 = np.array([[30], [40]])

# These are static obstacles we present to the robot
obs = np.hstack((const_obs, const_obs2))
N = len(roro)
cent = {'cent_F1':[],'cent_F2':[],'a':3,'b':2,'AF1':0,'AF2':0}
a, ax1 = plt.subplots()

Top = 1

def get_pose(robots):
    '''
    Returns the position of robots
    '''
    x = np.zeros((2, len(robots)))
    for i in range(len(robots)):
        x[:,i] = robots[i].state['q'][:2].reshape(2,)
    return x


def get_form_cent(rbts):
    if len(rbts)<1:
        cent = np.array([5,5]).reshape(2,)
    else:
        pose = get_pose(rbts)
        cent = np.mean(pose,axis=1)
    return cent

def get_angle(poses):
    
    diff = poses[:,0] - poses[:,len(poses)-1]
        
    angle = np.arctan2(diff[1],diff[0])
    return degrees(angle)


def check_goal_reached(Robots):
    for i in range(N):
        #print(Robots[i].state['q'].shape[0])
        if np.allclose(Robots[i].state['q'],Robots[i].goal,rtol=0, atol=1e-07):
            return True
        else:
            return False



def f_control(N,rbts,rbts1):

    tt = 0
    while not check_goal_reached(rbts):

        tt = tt +1
        for i in range(N):          
            cent['cent_F1'] = get_form_cent(rbts)
            #print(cent['cent_F1'])
            
            cent['cent_F2'] = get_form_cent(rbts1)
            #print(cent['cent_F2'])

            cent['AF1'] = get_angle(get_pose(rbts))
            cent['AF2'] = get_angle(get_pose(rbts1))

            #print(cent['cent_F2'])
            
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


                rbts[i].robot_step(obs,roro,i,i,7,L2,weights_rec2,e1,cent['cent_F2'],cent['AF2'])
                rbts1[i].robot_step(obs,roro1,i,i,7,L2,weights_rec2,e1,cent['cent_F1'],cent['AF1'])
                
            

        if tt%50 ==0:
            print(tt)
            plt.cla()

            for robot in rbts12:
                plot_step(robot.state_hist,ax1,obs,robot.n_fcentre,robot.angle)
            
            plt.pause(0.12)

    for robot in rbts12:
    
        robot.ecbf.new_plt_h(1)
    
        #robot.ecbf.dist_plot()

    
if __name__ =="__main__":

    f_control(N,roro,roro1)