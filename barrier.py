import numpy as np
import matplotlib.pyplot as plt
import time
from cvxopt import matrix, solvers
import functions

K = 1
# Laplacian matrix for a square/Rectangle shape with 4 robots|| Constarints for 2 and 4



# Class Defining our CBF control

class ebcf_control:
    def __init__(self,state,goal,id):
        self.state = state
        self.goal = goal
        self.h = {'q': [],
                    'h':[]}
        self.id = id

        self.dist = []
    
    # Compute h for the chosen case wrt to id provided
    def compute_h(self,obs,Robots,n,r,id,L,weights,e,centre,angle,c_vel):
        if id ==1:
            # Implement barrier for only Obstacles
            h = functions.compute_hobs(obs,self.state)
        if id == 2:
            # Implement barrier for only formation
            h = functions.compute_hf(self.state,Robots,n,r,L,weights,e)
        if id == 3:
            #Implement for both obstacles and formation
            h = functions.compute_hf_3(obs,self.state,Robots,n,r,L,weights,e)
        if id == 4:
            h = functions.compute_hf_2(obs,self.state,Robots,n,r,L,weights,e)

        if id == 5:
            h = functions.compute_hf_1(obs,self.state,Robots,n,r,L,weights,e)

        if id ==6 :
            #Implement for both obstacles and formation
            h = functions.compute_hf_4(obs,self.state,Robots,n,r,L,weights,e,centre,angle)
        if id ==7:
            h= functions.compute_hf_7(obs,self.state,Robots,n,r,L,weights,e,centre,angle,c_vel)
        if id ==8:
            h = functions.compute_hf_Form(self.state,3,2,angle,centre)
        return h



    # Compute G for the chosen case wrt to id provided
    def compute_G(self,obs,Robots,n,r,id,L,centre,angle):
        if id ==1:
            # Implement barrier for only Obstacles
            G = functions.compute_A_obs(self.state,obs)
        if id == 2:
            # Implement barrier for only formation
            G = functions.compute_Gf(self.state,Robots,n,r,L)
        if id == 3:
            #Implement for both obstacles and formation
            G = functions.compute_Gf_3(obs,self.state,Robots,n,r,L)
        if id == 4:
            #Implement for both obstacles and formation
            G = functions.compute_Gf_2(obs,self.state,Robots,n,r,L)

        if id ==5:
            G = functions.compute_Gf_l(obs,self.state,Robots,n,r,L)

        if id ==6:
            G = functions.compute_Gf_4(obs,self.state,Robots,n,r,L,centre,angle)
        if id ==7:
            # Implement barrier for only Obstacles
            G = functions.compute_Gf_7(obs,self.state,Robots,n,r,L,centre,angle)
        if id ==8:
            G = functions.compute_A_F(self.state,3,2,centre,angle)

        
        return G
    
    def compute_nom(self):
        u_nom = K*((self.state["q"][:2]).T-self.goal.T )
        return u_nom

    def compute_safe(self,obs,Robots,n,r,id,L,weights,e,centre,angle,c_vel):

        #print('This is i: ',i)
        P = np.array([[2, 0],[0, 2]])
        q = 2*self.compute_nom().reshape(2,)
        h = self.compute_h(obs,Robots,n,r,id,L,weights,e,centre,angle,c_vel)

        self.h['h'].append(h)
        #self.dist.append(functions.calc_dist(self.state,Robots,n,r,L))

        G = self.compute_G(obs,Robots,n,r,id,L,centre,angle)

        #print('This is G', G)
        #exit()
        
        sol = solve_qp(P,q,G,h)
        u_st = sol['x']

        #print('This is h: ',h)
        """ try:
            sol = solve_qp(P,q,G,h)
            u_st = sol['x']
        except ValueError:
            u_st = matrix([0,0])
            print('Robot {} failed'.format(i+1)) """
        return u_st

    # Creates respective arrays for h(x) values to be stored for plotting
    def create_h_arr(self):
        arr_list =[]
        #print('These are number of values in one h(x) array',len(self.h['h'][0]))
        for i in range(len(self.h['h'][0])):
            arr_list.append(np.zeros((len(self.h['h']),1)))
        return arr_list
    
    # Creates respective arrays for h(x) values to be stored for plotting
    def create_d_arr(self):
        arr_list =[]
        #print('These are number of values in one distance array',len(self.dist[0]))
        for i in range(len(self.dist[0])):
            arr_list.append(np.zeros((len(self.dist),1)))
        return arr_list
    # Function to plot the h(x) fucntions
    def new_plt_h(self,id):
        h_list= self.create_h_arr()
        plt.figure(2)
        plt.xlabel("Time(t)")
        plt.ylabel("h(x)")
        plt.title('Plot of h(x) array with {} values for robot {}'.format(len(self.h['h'][0]),self.id + 1))
        #print(self.h['h'][0])
        for i in range(len(h_list)):
            for j in range(len(self.h['h'])):
                h_list[i][j] = self.h['h'][j][i]
        for i in range(len(h_list)):
            plt.plot(h_list[i])

        if id ==2:
            d_list = self.create_d_arr()
            plt.figure(3)
            plt.xlabel("Time(t)")
            plt.ylabel("h(x)")
            plt.title('Plot of distance of robot {} w.rt to othe robots'.format(self.id))
        #print('This is the distance of robot {} w.rt to othe robots'.format(self.id),self.dist[0])
            for i in range(len(d_list)):
                for j in range(len(self.dist)):
                    d_list[i][j] = self.dist[j][i]

            for i in range(len(d_list)):
                plt.plot(d_list[i])


        plt.show()
    
    def dist_plot(self):
        
        plt.show()


def solve_qp(P,q,G,h):
    P = matrix(P,tc='d')
    q = matrix(q,tc='d')
    G = matrix(G.astype(np.double),tc='d')
    h = matrix(h.astype(np.double),tc='d')
    solvers.options['show_progress'] = False
    Sol = solvers.qp(P,q,G,h)
    
    return Sol

