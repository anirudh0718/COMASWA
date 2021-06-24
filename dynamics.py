import numpy as np



# Constants and Settings
dt = 0.01 # 10ms
tmax = 5 # simulation duration in seconds

# Set initial, desired position, and obstacle
Ds = 0.1
init_pos = np.array([0, 0])



class SingleDynamics:
    def __init__(self,init_pos):
        self.q = init_pos
        self.state = {"q": self.q}

    def updateDictState(self):
        # Update state dictionary
        self.state["q"] = self.q
        
        return self.state

    def step(self,u):
        
        pos = self.q

        #print('this is pos shape',pos.shape)

        pos_next = pos + dt*u

        #print('This is dt*u',dt*u)

        #print('This is pos_next',pos_next,'This is pos_next shape',pos_next.shape,'This is u shape',np.ndarray.flatten(np.array(u)).shape)
        self.q = pos_next.reshape(2,)

        return self.updateDictState()

        