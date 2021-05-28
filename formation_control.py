import numpy as np
from robot import Robot_Sim
from matplotlib.patches import Ellipse
import  matplotlib.pyplot as plt
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
def get_pose(robots):
    x = np.zeros((2, len(robots)))
    for i in range(len(robots)):
        x[:,i] = robots[i].state['q'][:2].reshape(2,)
    return x
#TODO Formation control with inherent cbf 
# Needed weight matrix 
# control law dxi = 1/4((xi-xj)^2 -d^2)(xj-xi)

# Laplacian matrix for a square shape with 4 robots
L = np.array([
    [3, -1, -1, -1],
    [-1, 2, -1, -1],
    [-1, -1, 3, -1],
    [-1, -1 , -1, 2]]) 
 #L = D-A 
""" 
L = np.array([[1, -1],
    [-1, 1]]) """

d = 1.2
ddiag =np.sqrt(2)*d

F_gain = 10

# Weight matrix

weights = np.array([
    [0, d,ddiag,d],
    [d, 0, d, ddiag],
    [ddiag, d, 0, d],
    [d, ddiag,d, 0]
])
#weights = np.array([[0, d],[d, 0]])
# Experiment constants
iterations = 10
N = 4

def plot_step(state_hist, plot_handle):
    state_hist_plot = np.array(state_hist)


    plot_handle.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], '8k') # current
    """ for i in range(new_obs.shape[1]):
        plot_handle.plot(new_obs[0, i], new_obs[1, i], '8k') # obs """
    

    ell = Ellipse((state_hist_plot[-1, 0], state_hist_plot[-1, 1]), 0.3, 0.3, 0)
    ell.set_alpha(0.3)
    ell.set_facecolor(np.array([1, 0, 0]))
    
    plot_handle.add_artist(ell)

#Robot 1
x_init1 = np.array([3, -5])
goal_init1 =np.array([-6, 4])
robot1 = Robot_Sim(x_init1, goal_init1, 0)

### Robot 2

x_init2 =np.array([5, 3])
goal_init2 =np.array([[4], [-6]])
Robot2 = Robot_Sim(x_init2, goal_init2,1)

### Robot 3

x_init3 =np.array([2, 1])
goal_init3 =np.array([[4], [-6]])
Robot3 = Robot_Sim(x_init3, goal_init3,2)

### Robot 4

x_init4 =np.array([1, 2])
goal_init4 =np.array([[4], [-6]])
Robot4 = Robot_Sim(x_init4, goal_init4,3)

Robots =[robot1,Robot2,Robot3,Robot4] #,robot2]
u_star =[]
poses = []
poses = get_pose(Robots)

a, ax1 = plt.subplots()
    
for tt in range(20000):

    dxi = np.zeros((2, N))
    for i in range(N):
        for j in topological_neighbors(L,i):
            error = Robots[j].state['q'][:2].reshape(2,1) - Robots[i].state['q'][:2].reshape(2,1)
            dxi[:, i] += (1*(np.power(np.linalg.norm(error), 2)- np.power(weights[i, j], 2))*error ).reshape(2,)

        Robots[i].robot_step_n(dxi[:,i])

        if(tt % 50 == 0):
            print(tt)
            plt.cla()
            for robot in Robots:
                
                plot_step(robot.state_hist,ax1)
                #print(robot.state_hist)

            plt.pause(1)

    #print('This is the dxi values',dxi)

    
    diff = (Robots[0].state['q']) - Robots[1].state['q']
    diff2 = (Robots[0].state['q']) - Robots[2].state['q']
    diff3 = (Robots[0].state['q']) - Robots[3].state['q']

    """ print('1 and 2',diff)
    print('1 aND 3',diff2) """


    print('Distance between robot 1 and robot 2', np.sqrt(np.power(diff[0],2) + (np.power(diff[1],2))))

    print('Distance between robot 1 and robot 3', np.sqrt(np.power(diff2[0],2) + (np.power(diff2[1],2))))

    print('Distance between robot 1 and robot 4', np.sqrt(np.power(diff3[0],2) + (np.power(diff3[1],2))))


    









