from matplotlib.patches import Ellipse
import numpy as np




def plot_step(state_hist, plot_handle,new_obs):
    state_hist_plot = np.array(state_hist)

    plot_handle.set_xlim([-10, 10])
    plot_handle.set_ylim([-10, 10])

    plot_handle.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], '8k') # current
    for i in range(new_obs.shape[1]):
        plot_handle.plot(new_obs[0, i], new_obs[1, i], '8k') # obs
    

    ell = Ellipse((state_hist_plot[-1, 0], state_hist_plot[-1, 1]), 2, 2, 0)
    ell.set_alpha(0.3)
    ell.set_facecolor(np.array([1, 0, 0]))
    
    plot_handle.add_artist(ell)
    plot_handle.axis('equal')
    