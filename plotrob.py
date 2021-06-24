from matplotlib.patches import Ellipse
import numpy as np




def plot_step(state_hist, plot_handle):
    state_hist_plot = np.array(state_hist)

    plot_handle.set_xlim([-5, 5])
    plot_handle.set_ylim([-5, 5])

    plot_handle.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], '8k') # current

    

    ell = Ellipse((state_hist_plot[-1, 0], state_hist_plot[-1, 1]), 0.3, 0.3, 0)
    ell.set_alpha(0.3)
    ell.set_facecolor(np.array([1, 0, 0]))
    
    plot_handle.add_artist(ell)
    #plot_handle.axis('equal')
    