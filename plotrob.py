from matplotlib.patches import Ellipse

import numpy as np




def plot_step(state_hist, plot_handle,new_obs,centre,angle):
    state_hist_plot = np.array(state_hist)
    centre_hist = np.array(centre)
    angle_hist = np.array(angle)
    #print(angle)
    k = 25

    plot_handle.set_xlim([-k, k])
    plot_handle.set_ylim([-k, k])

    plot_handle.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], '8k') # current

    """ plot_handle.plot(centre_hist[:, 0], centre_hist[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(centre_hist[-1, 0], centre_hist[-1, 1], '8k') """ # current

    
    for i in range(new_obs.T.shape[1]):
        #print('This is obstacle{}'.format(i),new_obs[i,0],new_obs[i,1])
        plot_handle.plot(new_obs[i,0],new_obs[i,1], '8k') # obs

        ell = Ellipse((new_obs[0,0],new_obs[0,1]), 4, 4, 0)
        ell2 = Ellipse((new_obs[1,0],new_obs[1,1]), 4, 4, 0)
        ell.set_alpha(0.3)
        ell.set_facecolor(np.array([1, 0, 0]))
        ell2.set_alpha(0.3)
        ell2.set_facecolor(np.array([1, 0, 0]))

    #print((centre_hist))
    ell3 = Ellipse((centre_hist[-1, 0], centre_hist[-1, 1]), 6, 4, angle_hist[-1])
    #plot_handle.plot(centre_hist[:, 0],centre_hist[:, 1],'2k')
    ell3.set_alpha(0.3)
    ell3.set_facecolor(np.array([1, 0, 0]))
    
    #plot_handle.add_artist(ell)
    #plot_handle.add_artist(ell2)
    plot_handle.add_artist(ell3)
    #plot_handle.axis('equal')
    