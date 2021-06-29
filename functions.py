import numpy as np
import matplotlib.pyplot as plt
import time
from cvxopt import matrix, solvers

# Number of robots
N = 4

# Laplacian matrix for a square/Rectangle shape with 4 robots
L = np.array([
    [3, -1, -1, -1],
    [-1, 2, -1, 0],
    [-1, -1, 3, -1],
    [-1, 0 , -1, 2]]) 

# Distance between the robot and obstacle (minimal)
Ds = 5

e = 0.4


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

# Formation Distance for rectangle shape
Df_l = 3
Df_b = 2

ddiag_rec =np.sqrt(np.power(Df_l,2) + np.power(Df_b,2))  # 2.2360

weights_rec = np.array([
    [0, Df_b,ddiag_rec,Df_l],
    [Df_b, 0, Df_l, 0],
    [ddiag_rec, Df_l, 0, Df_b],
    [Df_l, 0,Df_b, 0]
])




def topological_neighbors(L, agent):
    """ Returns the neighbors of a particular agent using the graph Laplacian

    L: NxN numpy array (representing the graph Laplacian)
    agent: int (agent: 0 - N-1)

    -> 1xM numpy array (with M neighbors)
    """
    #Check user input types
    assert isinstance(L, np.ndarray), "In the topological_neighbors function, the graph Laplacian (L) must be a numpy ndarray. Recieved type %r." % type(L).__name__
    assert isinstance(agent, int), "In the topological_neighbors function, the agent number (agent) must be an integer. Recieved type %r." % type(agent).__name__
    
    #Check user input ranges/sizes
    assert agent >= 0, "In the topological_neighbors function, the agent number (agent) must be greater than or equal to zero. Recieved %r." % agent
    assert agent <= L.shape[0], "In the topological_neighbors function, the agent number (agent) must be within the dimension of the provided Laplacian (L). Recieved agent number %r and Laplactian size %r by %r." % (agent, L.shape[0], L.shape[1])

    row = L[agent, :]
    #print('This isn row = L[agent,:]', row)
    row[agent]=0
    # Since L = D - A
    #print('This is np.where(row != 0)',np.where(row != 0))
    return np.where(row != 0)[0]
#-----------------------------------------------------------------------------------------------------
# Compute h for only Obstacle
def compute_hobs(obst,state):
    h = np.zeros((obst.shape[1], 1))
    for i in range(obst.shape[1]):
        sub = np.atleast_2d(state["q"][:2]).T - obst[i, :].T.reshape(2,1)
        xo = sub[0]
        yo = sub[1]
        h[i] = np.power(xo,2)+ np.power(yo,2)- np.power(Ds,2) 
    return 50*h

# Compute hf for only greater than inequality
def compute_hf_g(state,Robots,i):
    hf = np.zeros((len(topological_neighbors(L,i)), 1))
    idx = 0    
    for j in topological_neighbors(L,i):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            
        hf[idx]= (np.power(xo,2)+ np.power(yo,2)) - np.power(weights_rec[i,j] - e,2)
        idx += 1

    return hf
# Compute hf for only lesser than inequality
def compute_hf_l(state,Robots,i):
    hf = np.zeros((len(topological_neighbors(L,i)), 1))
    idx = 0
    for j in topological_neighbors(L,i):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            #print(xo,yo)
        hf[idx]= np.power(weights_rec[i,j] + e,2) - (np.power(xo,2)+ np.power(yo,2))
        idx += 1

    return hf

# Compute h for only formation

def compute_hf(state,Robots,i):
    com_h = np.vstack((compute_hf_g(state,Robots,i),compute_hf_l(state,Robots,i)))
    return com_h

#Compute h for obs and Formation
def compute_hf_3(obs,state,Robots,i):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,i),compute_hf_l(state,Robots,i)))
    return com_h


#Compute h for obs and Formation_greater case
def compute_hf_2(obs,state,Robots,i):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,i)))
    return com_h

#Compute h for obs and Formation_lesser case
def compute_hf_1(obs,state,Robots,i):
    com_h = compute_hf_l(state,Robots,i)
    #com_h = np.vstack((compute_hobs(obs,state),compute_hf_l(state,Robots,i)))
    return com_h


# Compute G for only formation
def compute_Gf(state,Robots,i):
    Af = compute_Af(state,Robots,i)
    Af_co = np.vstack((Af,-1*Af))
    return Af_co

# Compute Gf for only form_greater case
def compute_Gf_g(state,Robots,i):
    Af = compute_Af(state,Robots,i)
    return Af

#Compute G for obs and Formation
def compute_Gf_3(obs,state,Robots,i):
    A = compute_A_obs(state,obs)
    Af = compute_Gf(state,Robots,i)
    Af_3 = np.vstack((A,Af))
    return Af_3

# Compute G for obs and form_greater case

def compute_Gf_2(obs,state,Robots,i):
    A = compute_A_obs(state,obs)
    Af = compute_Gf_g(state,Robots,i)
    Af_3 = np.vstack((A,Af))
    return Af_3
def compute_Gf_l(obs,state,Robots,i):
    #A = compute_A_obs(state,obs)
    Af = compute_Gf_g(state,Robots,i)
    #Af_3 = np.vstack((A,-Af))
    return -1*Af

# Compute G for only obstacle
def compute_A_obs(state,obst):
    A = np.empty((0,2))
    for i in range(obst.shape[1]):
                
        sub = np.atleast_2d(state["q"][:2]).T - obst[i, :].T.reshape(2,1)
        xo = sub[0]
        yo = sub[1]

        atmp = np.array([np.hstack((xo, yo))])
        A =np.array(np.vstack((A,atmp)))
    return A

# Compute G for Formation
def compute_Af(state,Robots,i):
    A = np.empty((0,2))
        
    for j in topological_neighbors(L,i):
        sub =  Robots[j].state['q'][:2].reshape(2,1) - state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
        atmp = np.array([np.hstack((xo, yo))])
        A =np.array(np.vstack((A,atmp)))
    return A


# Calculate distance between the robots

def calc_dist(state,Robots,i):
    dist = np.zeros((len(topological_neighbors(L,i)), 1))
    diff ={}
    for j in topological_neighbors(L,i):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            
        diff[j]= (np.power(xo,2)+ np.power(yo,2))

    for m in range(len(diff.keys())):
        for key in diff.keys():
            dist[m] =  diff[key]
    return np.sqrt(dist)