import numpy as np
import matplotlib.pyplot as plt
import time
from cvxopt import matrix, solvers

# Number of robots
N = 4


Ds = 3




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
        #print('This is the obstacle {}'.format(i),obst[i, :].T.reshape(2,1))
        #exit()
        sub = np.atleast_2d(state["q"][:2]).T - obst[i, :].T.reshape(2,1)

        xo = sub[0]
        yo = sub[1]
        h[i] = np.power(xo,2)+ np.power(yo,2)- np.power(Ds,2) 
    return h

# Compute hf for only greater than inequality
def compute_hf_g(state,Robots,i,L,weights,e):
    hf = np.zeros((len(topological_neighbors(L,i)), 1))
    idx = 0    
    for j in topological_neighbors(L,i):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            
        hf[idx]= (np.power(xo,2)+ np.power(yo,2)) - np.power(weights[i,j] - e,2)
        idx += 1

    return hf
# Compute hf for only lesser than inequality
def compute_hf_l(state,Robots,i,L,weights,e):
    hf = np.zeros((len(topological_neighbors(L,i)), 1))
    idx = 0
    for j in topological_neighbors(L,i):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            #print(xo,yo)
        hf[idx]= np.power(weights[i,j] + e,2) - (np.power(xo,2)+ np.power(yo,2))
        idx += 1

    return hf

# Compute h for only formation

def compute_hf(state,Robots,i,L,weights,e):
    com_h = np.vstack((compute_hf_g(state,Robots,i,L,weights,e),compute_hf_l(state,Robots,i,L,weights,e)))
    return com_h

#Compute h for obs and Formation
def compute_hf_3(obs,state,Robots,i,L,weights,e):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,i,L,weights,e),compute_hf_l(state,Robots,i,L,weights,e)))
    return com_h


#Compute h for obs and Formation_greater case
def compute_hf_2(obs,state,Robots,i,L,weights,e):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,i,L,weights,e)))
    return com_h

#Compute h for obs and Formation_lesser case
def compute_hf_1(obs,state,Robots,i,L,weights,e):
    com_h = compute_hf_l(state,Robots,i,L,weights,e)
    #com_h = np.vstack((compute_hobs(obs,state),compute_hf_l(state,Robots,i)))
    return com_h


# Compute G for only formation
def compute_Gf(state,Robots,i,L):
    Af = compute_Af(state,Robots,i,L)
    Af_co = np.vstack((Af,-1*Af))
    return Af_co

# Compute Gf for only form_greater case
def compute_Gf_g(state,Robots,i,L):
    Af = compute_Af(state,Robots,i,L)
    return Af

#Compute G for obs and Formation
def compute_Gf_3(obs,state,Robots,i,L):
    A = -2*compute_A_obs(state,obs)
    Af = compute_Gf(state,Robots,i,L)
    Af_3 = np.vstack((A,Af))
    return Af_3

# Compute G for obs and form_greater case

def compute_Gf_2(obs,state,Robots,i,L):
    A = compute_A_obs(state,obs)
    Af = compute_Gf_g(state,Robots,i,L)
    Af_3 = np.vstack((A,Af))
    return Af_3
def compute_Gf_l(obs,state,Robots,i,L):
    #A = compute_A_obs(state,obs)
    Af = compute_Gf_g(state,Robots,i,L)
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
def compute_Af(state,Robots,i,L):
    A = np.empty((0,2))
        
    for j in topological_neighbors(L,i):
        sub =  Robots[j].state['q'][:2].reshape(2,1) - state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
        atmp = np.array([np.hstack((xo, yo))])
        A =np.array(np.vstack((A,atmp)))
    return A


# Calculate distance between the robots

def calc_dist(state,Robots,i,L):
    dist = np.zeros((len(topological_neighbors(L,i)), 1))
    idx = 0
    for j in topological_neighbors(L,i):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            

        dist[idx]= (np.power(xo,2)+ np.power(yo,2))
        idx += 1

    return np.sqrt(dist)