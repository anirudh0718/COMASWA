from matplotlib.pyplot import pink
import numpy as np
from dynamics import SingleDynamics
from barrier import ebcf_control



class Robot_Sim():
    def __init__(self, x_init, goal_init, robot_id):
        self.id = robot_id
        self.state = {"q": x_init
                }
        self.dyn = SingleDynamics(x_init)
        self.goal = goal_init
        self.ecbf = ebcf_control(self.state, self.goal)


        self.state_hist = []
        self.state_hist.append(self.state["q"])

        self.new_obs = np.array([[1], [1]])
    def robot_step(self, new_obs):
        u_hat_acc = self.ecbf.compute_safe(obs=new_obs)
        
        u_hat_acc = np.ndarray.flatten(np.array(u_hat_acc)) 
        """ print((u_hat_acc).shape)
        exit() """
         # acceleration
        assert(u_hat_acc.shape == (2,))
        

        self.state = self.dyn.step(u_hat_acc)
        self.ecbf.state = self.state
        self.state_hist.append(self.state["q"])
        return u_hat_acc

    def robot_step_n(self, u_nom):
        
        

        self.state = self.dyn.step(u_nom)
        self.ecbf.state = self.state
        self.state_hist.append(self.state["q"])
        return u_nom

    def update_obstacles(self, robots, obs):
        obst = []
        for robot in robots:
            if robot.id == self.id:
                continue
            #print('This is the robot state',robot.state["q"][:2].reshape(2,1))
            obst.append(robot.state["q"][:2].reshape(2,1))
            #print('')
            #print('This is the updated obstacles',obst)
        if not len(obs):
            return {"obs":obst}
        #print('The first element of obs',obs[:,0])
        for i in range(obs.shape[1]):

            obst.append(obs[:,i].reshape(2,1))
        
        obstacles = {"obs":obst}
        return obstacles