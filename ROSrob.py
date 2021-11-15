import numpy as np
from dynamics import SingleDynamics
from barrier import ebcf_control



class Robot_ROS():
    def __init__(self, x_init, goal_init, robot_id,id):
        self.id = robot_id
        self.state = {"q": x_init
                }
        self.dyn = SingleDynamics(x_init)
        self.goal = goal_init
        self.ecbf = ebcf_control(self.state, self.goal,robot_id)


        self.state_hist = []
        self.state_hist.append(self.state["q"])

        self.p_fcentre = []
        self.n_fcentre = []
        self.angle = []
        self.vel = np.zeros([2,])
        self.form_id =id


    # Calcute safe velocity for the robots and update its dynamics
    def robot_step(self, obs,Robots,n,r,id,L,weights,e,centre,angle,c_vel,a_dot):
        #self.p_fcentre.append(centre.reshape(2,))
        """ if id ==6 and i > 3:



            i == 4 """
        
        u_hat_acc = self.ecbf.compute_safe(obs,Robots,n,r,id,L,weights,e,centre,angle,c_vel,a_dot)
        
        u_hat_acc = np.ndarray.flatten(np.array(u_hat_acc)) 

        #print('This is the Velocity',np.sqrt(np.power(u_hat_acc[0],2)+np.power(u_hat_acc[1],2) ))
        #print('This the ustar: ',u_hat_acc)
        assert(u_hat_acc.shape == (2,))
        self.vel = u_hat_acc

        #self.state = self.dyn.step(u_hat_acc)
        #self.ecbf.state = self.state
        #self.state_hist.append(self.state["q"])
        
        #self.n_fcentre.append(centre.reshape(2,))
        #self.angle.append(angle)
        #self.vel.append(u_hat_acc)

        return u_hat_acc


    # If given a new goal update the initial goal position to the new one
    def update_goal(self,goal):
        self.goal = goal
        self.ecbf.goal = goal

    
    # Update the robot dynamics when safe velocity is provided
    def robot_step_n(self, u_nom):
        
        

        self.state = self.dyn.step(u_nom)
        self.ecbf.state = self.state
        self.state_hist.append(self.state["q"])
        return u_nom

    # Update the robots dynamics with only nominal velocity
    def robot_step_g(self):
        
        u_nom = 1*(self.goal.T - (self.state["q"][:2]).T )

        self.state = self.dyn.step(u_nom)
        self.ecbf.state = self.state
        self.state_hist.append(self.state["q"])
        return u_nom
    
    def update_state(self,pose):
        #print('pose of robot',pose)
        self.state['q'] = np.array([pose.x,pose.y])
        
        self.ecbf.state = self.state
        #print('Updated ECBF state of robot',self.ecbf.state['q'])
        return self.state

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