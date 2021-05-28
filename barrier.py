import numpy as np
import matplotlib.pyplot as plt
import time
from cvxopt import matrix, solvers

Ds = 2 # Distance between the robot and obstacle (minimal)
K = 1
class ebcf_control:
    def __init__(self,state,goal):
        self.state = state
        self.goal = goal
    

    def compute_h(self,obst):
        h = np.zeros((obst.shape[1], 1))
        for i in range(obst.shape[1]):
            sub = np.atleast_2d(self.state["q"][:2]).T - obst[i, :].T.reshape(2,1)
            xo = sub[0]
            yo = sub[1]
            h[i] = np.power(xo,2)+ np.power(yo,2)- Ds
        return h
    
    def compute_A(self,obst):
            A = np.empty((0,2))

            #print('This is obst',obst[0,:])

            #print('This is the shape', obst.shape)
            #exit()

            for i in range(obst.shape[1]):
                
                sub = np.atleast_2d(self.state["q"][:2]).T - obst[i, :].T.reshape(2,1)
                xo = sub[0]
                yo = sub[1]

                atmp = np.array([np.hstack((xo, yo))])
                A =np.array(np.vstack((A,atmp)))
            return A

    def compute_nom(self):
        """ print('This is robot pos ',self.state["q"][:2].T.shape)
        print('This is goal pos', self.goal.T,'This is the shape',self.goal.T.shape)
        print('This is the difference',np.atleast_2d(self.state["q"][:2]).T - self.goal.T) """
        u_nom = K*((self.state["q"][:2]).T - self.goal.T)
        return u_nom

    def compute_safe(self,obs):
        A = self.compute_A(obs)
        h = self.compute_h(obs)

        P = np.array([[2, 0],[0, 2]])
        G = -2*(A)
        """ print('This is u nom',self.compute_nom())
        exit() """
        q = 2*self.compute_nom().reshape(2,)
        print('This is q',q.shape)

        sol = solve_qp(P,q,G,h)

        u_st = sol['x']

        return u_st
    
    def compute_safe_f(self,obs,u):
        A = self.compute_A(obs)
        h = self.compute_h(obs)

        P = np.array([[2, 0],[0, 2]])
        G = -2*(A)
        """ print('This is u nom',self.compute_nom())
        exit() """
        q = -2*u
        #print('This is q',q)

        print(A.shape,h.shape,P.shape,G.shape,q.shape)

        #exit()

        sol = solve_qp(P,q,G,h)

        u_st = sol['x']

        return u_st






def solve_qp(P,q,G,h):
    P = matrix(P,tc='d')
    q = matrix(q,tc='d')
    G = matrix(G.astype(np.double),tc='d')
    h = matrix(h.astype(np.double),tc='d')
    solvers.options['show_progress'] = False
    Sol = solvers.qp(P,q,G,h)
    
    return Sol

        


            




