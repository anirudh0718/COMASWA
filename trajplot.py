import numpy as np
import matplotlib.pyplot as plt



class TrajHistory():
    """ Keeps track of state and input history for plotting. """
    def __init__(self):
        self.hist_x = [] # q_x
        self.hist_y = [] # q_y
        self.hist_ux = [] # u_x
        self.hist_uy = [] # u_y
        
        self.hist_q = []
        self.hist_qdot = []
        self.hist_u = []
        self.hist_t = []

    def update_history(self, state, u, dt):
        """Appends current state and desired theta for plotting."""
        q = state["q"]
        if not self.hist_q:
            qdot = np.array([0, 0])
            t = 0
        else:
            qdot = (q - self.hist_q[-1]) / dt
            t = self.hist_t[-1] + dt
        self.hist_x.append(q[0])
        self.hist_y.append(q[1])
        self.hist_ux.append(u[0])
        self.hist_uy.append(u[1])

        self.hist_q.append(q)
        self.hist_qdot.append(qdot)
        self.hist_u.append(u)
        self.hist_t.append(t)
        
    def plot2DTraj(self):
        # Initialize the plot
        fig = plt.figure(figsize=(12,5), dpi= 100)
        ax = fig.add_subplot(2, 2, (1,3))
        ax_x_traj = fig.add_subplot(2, 2, 2)
        ax_y_traj = fig.add_subplot(2, 2, 4)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        # ax_x_traj.set_xlabel("t [s]")
        ax_x_traj.set_ylabel("x [m]")
        ax_y_traj.set_xlabel("t [s]")
        ax_y_traj.set_ylabel("y [m]")
        #ax.grid()
        ax_x_traj.grid()
        ax_y_traj.grid()

        # Visualize Trajectory
        ax.plot(self.hist_x, self.hist_y)
        ax_x_traj.plot(self.hist_t, self.hist_x)
        ax_y_traj.plot(self.hist_t, self.hist_y)
                
        return {"2D": ax, "x_traj": ax_x_traj, "y_traj": ax_y_traj}
        
    def plotInput(self):
        # Initialize the plot
        fig = plt.figure(figsize=(12,2), dpi= 100)
        ax_u_x = fig.add_subplot(1, 2, 1)
        ax_u_y = fig.add_subplot(1, 2, 2)
        ax_u_x.set_xlabel("t [s]")
        ax_u_x.set_ylabel("u_x [m/s]")
        ax_u_y.set_xlabel("t [s]")
        ax_u_y.set_ylabel("u_y [m/s]")
        ax_u_x.grid()
        ax_u_y.grid()

        # Visualize the changes of Input
        ax_u_x.plot(self.hist_t, self.hist_ux)
        ax_u_y.plot(self.hist_t, self.hist_uy)
        
        return {"u_x": ax_u_x, "u_y": ax_u_y}
