import numpy as np
from robot import Robot_Sim
from matplotlib.patches import Ellipse
import  matplotlib.pyplot as plt


#Distance between the robots, length of side of square
d = 2
l = 0.5
b = 0.25

l1 = 0.5
b1 = 0.25

def get_rob_gposes(centroid):
    P1_x = centroid[0]- d/2
    P1_y = centroid[1] +d/2

    P3 = np.array([P1_x,P1_y])

    P2_x = centroid[0]+ d/2
    P2_y = centroid[1] +d/2
    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] - d/2
    P3_y = centroid[1] - d/2
    P4 = np.array([P3_x,P3_y])

    P4_x = centroid[0]+ d/2
    P4_y = centroid[1] -d/2
    P1 = np.array([P4_x,P4_y])



    return P1,P2,P3,P4

def get_pose(robots):
    '''
    Returns the position of robots
    '''
    x = np.zeros((2, len(robots)))
    for i in range(len(robots)):
        x[:,i] = robots[i].state['q'][:2].reshape(2,)
    return x

def get_rob_rec_pos(centroid):
    l =0.5
    b = 0.25
    P1_x = centroid[0]- l/2
    P1_y = centroid[1] +b/2

    P3 = np.array([P1_x,P1_y])

    P2_x = centroid[0]+ l/2
    P2_y = centroid[1] +b/2
    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] - l/2
    P3_y = centroid[1] - b/2
    P4 = np.array([P3_x,P3_y])

    P4_x = centroid[0]+ l/2
    P4_y = centroid[1] -b/2
    P1 = np.array([P4_x,P4_y])

    return P1,P2,P3,P4

def get_rob_rec_pos_ros(centroid):
    P1_x = centroid[0]- l1/2
    P1_y = centroid[1] +b1/2

    P3 = np.array([P1_x,P1_y])

    P2_x = centroid[0]+ l1/2
    P2_y = centroid[1] +b1/2
    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] - l1/2
    P3_y = centroid[1] - b1/2
    P4 = np.array([P3_x,P3_y])

    P4_x = centroid[0]+ l1/2
    P4_y = centroid[1] -b1/2
    P1 = np.array([P4_x,P4_y])

    return P1,P2,P3,P4

def get_rob_rec_pos_far(centroid):
    l = 10
    b = 10
    P1_x = centroid[0]- l/2
    P1_y = centroid[1] +b/2

    P3 = np.array([P1_x,P1_y])

    P2_x = centroid[0]+ l/2
    P2_y = centroid[1] +b/2
    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] - l/2
    P3_y = centroid[1] - b/2
    P4 = np.array([P3_x,P3_y])

    P4_x = centroid[0]+ l/2
    P4_y = centroid[1] -b/2
    P1 = np.array([P4_x,P4_y])

    return P1,P2,P3,P4

def get_turned_rec(centroid):
    P1_x = centroid[0]- b/2
    P1_y = centroid[1] +l/2

    P3 = np.array([P1_x,P1_y])

    P2_x = centroid[0]+ b/2
    P2_y = centroid[1] +l/2
    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] - b/2
    P3_y = centroid[1] - l/2
    P4 = np.array([P3_x,P3_y])

    P4_x = centroid[0]+ b/2
    P4_y = centroid[1] -l/2
    P1 = np.array([P4_x,P4_y])

    return P1,P2,P3,P4

def get_turn_orient(centroid):
    
    P1_x = centroid[0] - (b/2)
    P1_y = centroid[1] - (l/2)
    
    P1 = np.array([P1_x,P1_y])

    P2_x = centroid[0] +b/2
    P2_y = centroid[1] - l/2

    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] +b/2
    P3_y = centroid[1] + l/2

    P3 = np.array([P3_x,P3_y])

    P4_x = centroid[0] -b/2
    P4_y = centroid[1] + l/2

    P4 = np.array([P4_x,P4_y])

    return P1,P2,P3,P4


def get_turn_orient_ros(centroid,l,b):
    
    P1_x = centroid[0] -b/2
    P1_y = centroid[1] - l/2

    P1 = np.array([P1_x,P1_y])

    P2_x = centroid[0] +b/2
    P2_y = centroid[1] - l/2

    P2 = np.array([P2_x,P2_y])

    P3_x = centroid[0] +b/2
    P3_y = centroid[1] + l/2

    P3 = np.array([P3_x,P3_y])

    P4_x = centroid[0] -b/2
    P4_y = centroid[1] + l/2

    P4 = np.array([P4_x,P4_y])

    return P1,P2,P3,P4