from two_robots_form import Df_b
import numpy as np




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

Df_l = 2
Df_b = 1

ddiag_rec =np.sqrt(np.power(Df_l,2) + np.power(Df_b,2)) 
LPR = np.array([
    [2, 0, -1, 0],
    [-1, 3, -1, 0],
    [-1, 0, 3, 0],
    [0, -1 , -1, 2]]) 

weights_PR = np.array([
    [0, 0,Df_b,0],
    [Df_l, 0, ddiag_rec, Df_b],
    [Df_b, 0, 0, 0],
    [0, Df_b,Df_l, 0]
])
print(topological_neighbors(LPR,0)+1)
print(weights_PR[0,2])