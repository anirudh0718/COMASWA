import numpy as np
import matplotlib.pyplot as plt
import time
from cvxopt import matrix, solvers
from numpy.lib import angle

# Number of robots
N = 4


Ds = 2




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
        t = 0.5
        #print('This is the obstacle {}'.format(i),obst[i, :].T.reshape(2,1))
        #exit()
        sub = np.atleast_2d(state["q"][:2]).T - obst[i, :].T.reshape(2,1)

        xo = sub[0]
        yo = sub[1]
        
        h[i] = np.power(xo,2)+ np.power(yo,2)- np.power(Ds,2) 
        #h[i] = np.power((xo*np.cos(t)-yo*np.sin(t)),2)/np.power(3,2) +  np.power((xo*np.sin(t)+yo*np.cos(t)),2)/np.power(2,2) - np.power(1,2) 
        
    return h

# Compute hf for only greater than inequality

def compute_hf_g(state,Robots,n,r,L,weights,e):
    hf = np.zeros((len(topological_neighbors(L,n)), 1))
    idx = 0    
    for j in topological_neighbors(L,n):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            
        hf[idx]= (np.power(xo,2)+ np.power(yo,2)) - np.power(weights[n,j] - e,2)
        idx += 1

    return hf
# Compute hf for only lesser than inequality
def compute_hf_l(state,Robots,n,r,L,weights,e):
    hf = np.zeros((len(topological_neighbors(L,n)), 1))
    idx = 0
    for j in topological_neighbors(L,n):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            #print(xo,yo)
        hf[idx]= np.power(weights[n,j] + e,2) - (np.power(xo,2)+ np.power(yo,2))
        idx += 1

    return hf

# Compute hf between formations

def compute_hf_Form(state,a,b,t,centre,c_vel):
    #print('This is the elliptical centre: ',centre)
    sub = np.atleast_2d(state["q"][:2]).T - centre.reshape(2,1)

    xo = sub[0]
    yo = sub[1]
    xobs = -sub[0]
    yobs = -sub[1]
    xalphobs= np.power(xo,2)*((2/a**2)*(np.cos(t)*(-np.sin(t))) + (2/b**2)*np.sin(t)*(np.cos(t))) +np.power(yo,2)*((2/a**2)*(np.cos(t)*(np.sin(t))) - (2/b**2)*np.sin(t)*(np.cos(t))) + 2*xo*yo*(1/a**2 - 1/b**2)*(np.power(np.cos(t),2)-np.power(np.sin(t),2))

    rel_vel = c_vel
    alpha_dot = rel_vel/1.5
    #velocity_factor = xobs*vel[0] + yobs*vel[1]
    #h =  np.power((xo*np.cos(t)-yo*np.sin(t)),2)/np.power(3,2) +  np.power((xo*np.sin(t)-yo*np.cos(t)),2)/np.power(2,2) - np.power(1,2) 
    h = (np.power(np.cos(t),2)/a**2+ np.power(np.sin(t),2)/b**2)*(np.power(xo,2)) + (np.power(np.sin(t),2)/a**2+np.power(np.cos(t),2)/b**2)*(np.power(yo,2)) + 2*xo*yo*(1/a**2 - 1/b**2) - 1
    """ for i in range(len(centre)):
        #print('This is the obstacle {}'.format(i),obst[i, :].T.reshape(2,1))
        #exit()
        

        
        #h[i] = np.power(xo,2)/np.power(3,2)+ np.power(yo,2)/np.power(2,2) - np.power(1,2) 
        h[i] =  """

    #print('This is th value of h: ',h)
    h =  h + xobs*rel_vel[0] + yobs*rel_vel[1] + xalphobs*alpha_dot

    return 0.05*h

# Compute h for only formation

def compute_hf(state,Robots,n,r,L,weights,e):
    com_h = np.vstack((compute_hf_g(state,Robots,n,r,L,weights,e),compute_hf_l(state,Robots,n,r,L,weights,e)))
    return com_h

#Compute h for obs and Formation
def compute_hf_3(obs,state,Robots,n,r,L,weights,e):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,n,r,L,weights,e),compute_hf_l(state,Robots,n,r,L,weights,e)))
    return com_h

#Compute h for obs(centre) and Formation
def compute_hf_4(obs,state,Robots,n,r,L,weights,e,centre,angle):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,n,r,L,weights,e),compute_hf_l(state,Robots,n,r,L,weights,e),compute_hf_Form(state,4,3,angle,centre)))
    return com_h

#Compute h for obs(centre) and Formation
def compute_hf_7(obs,state,Robots,n,r,L,weights,e,centre,angle,c_vel):
    a = 2
    vel_fac = np.array([a,a]).reshape(2,1)

    com_h = np.vstack((compute_hobs(obs,state),compute_hf_Form(state,3,2,angle,centre,c_vel),compute_hf_g(state,Robots,n,r,L,weights,e),compute_hf_l(state,Robots,n,r,L,weights,e),vel_fac,vel_fac))
    return com_h

#Compute h for obs and Formation_greater case
def compute_hf_2(obs,state,Robots,n,r,L,weights,e):
    com_h = np.vstack((compute_hobs(obs,state),compute_hf_g(state,Robots,n,r,L,weights,e)))
    return com_h

#Compute h for obs and Formation_lesser case
def compute_hf_1(obs,state,Robots,n,r,L,weights,e):
    com_h = compute_hf_l(state,Robots,n,r,L,weights,e)
    #com_h = np.vstack((compute_hobs(obs,state),compute_hf_l(state,Robots,i)))
    return com_h


# Compute G for only formation
def compute_Gf(state,Robots,n,r,L):
    Af = compute_Af(state,Robots,n,r,L)
    Af_co = np.vstack((Af,-1*Af))
    return Af_co

# Compute Gf for only form_greater case
def compute_Gf_g(state,Robots,n,r,L):
    Af = compute_Af(state,Robots,n,r,L)
    return Af

#Compute G for obs and Formation
def compute_Gf_3(obs,state,Robots,n,r,L):
    A = -2*compute_A_obs(state,obs)
    Af = compute_Gf(state,Robots,n,r,L)
    Af_3 = np.vstack((A,Af))
    return Af_3
#Compute G for obs and Formation
def compute_Gf_4(obs,state,Robots,n,r,L,centre,angle):
    A = -2*compute_A_obs(state,obs)
    Af = compute_Gf(state,Robots,n,r,L)
    AF = compute_A_F(state,4,3,centre,angle)
    Af_4 = np.vstack((A,Af,AF))
    return Af_4

# Single Elliptical and obstacle

def compute_Gf_7(obs,state,Robots,n,r,L,centre,angle):
    A = -2*compute_A_obs(state,obs)
    Af = compute_Gf(state,Robots,n,r,L)
    AF = compute_A_F(state,3,2,centre,angle)
    A1 = np.identity((2))
    A2 = -A1
    Af_4 = np.vstack((A,AF,Af,A1,A2))
    return Af_4
# Compute G for obs and form_greater case

def compute_Gf_2(obs,state,Robots,n,r,L):
    A = compute_A_obs(state,obs)
    Af = compute_Gf_g(state,Robots,n,r,L)
    Af_3 = np.vstack((A,Af))
    return Af_3
def compute_Gf_l(obs,state,Robots,n,r,L):
    #A = compute_A_obs(state,obs)
    Af = compute_Gf_g(state,Robots,n,r,L)
    #Af_3 = np.vstack((A,-Af))
    return -1*Af

# Compute G for only obstacle
def compute_A_obs(state,obst):
    A = np.empty((0,2))
    for i in range(obst.shape[1]):
                
        sub = np.atleast_2d(state["q"][:2]).T - obst[i, :].T.reshape(2,1)
        xo = 2*sub[0]
        yo = 2*sub[1]

        atmp = np.array([np.hstack((xo, yo))])
        A =np.array(np.vstack((A,atmp)))
    return A
# Compute G for only obstacle
def compute_A_F(state,a,b,centre,alpha):
    A = np.empty((0,2))

    sub = np.atleast_2d(state["q"][:2]).T - centre.reshape(2,1)
    xo = sub[0]
    yo = sub[1]

    r = 2*xo *(((np.cos(alpha))**2/a**2)+((np.sin(alpha))**2/b**2)) + 2*yo*(1/a**2 - 1/b**2)
    p = 2*yo*(((np.sin(alpha))**2/a**2) + ((np.cos(alpha))**2/b**2))+ 2*xo*(1/a**2 - 1/b**2)

    atmp = np.array([np.hstack((-r, -p))])
    A =np.array(np.vstack((A,atmp)))
    #for i in range(obst.shape[1]):
                  
        
    return A
# Compute G for Formation
def compute_Af(state,Robots,n,r,L):
    A = np.empty((0,2))
        
    for j in topological_neighbors(L,n):
        sub =  Robots[j].state['q'][:2].reshape(2,1) - state['q'][:2].reshape(2,1)
        xo = 2*sub[0]
        yo = 2*sub[1]
        atmp = np.array([np.hstack((xo, yo))])
        A =np.array(np.vstack((A,atmp)))
    return A


# Calculate distance between the robots

def calc_dist(state,Robots,n,r,L):
    dist = np.zeros((len(topological_neighbors(L,n)), 1))
    idx = 0
    for j in topological_neighbors(L,n):
        sub =  state['q'][:2].reshape(2,1) - Robots[j].state['q'][:2].reshape(2,1)
        xo = sub[0]
        yo = sub[1]
            

        dist[idx]= (np.power(xo,2)+ np.power(yo,2))
        idx += 1

    return np.sqrt(dist)