from matplotlib.patches import Ellipse

import numpy as np




def plot_step(state_hist, plot_handle,new_obs,centre,angle,gridlength,goal,id,time,robotid):
    state_hist_plot = np.array(state_hist)
    centre_hist = np.array(centre)
    angle_hist = np.array(angle)
    #print(angle)
    k = gridlength[0]
    p = gridlength[1]
    time_str = 'RUNTIME: '+str(time)+' SECS'
    text_kwargs = dict(ha='center', va='center', fontsize=12, color='C1')
    plot_handle.set_xlim([-2.5, 2.5])
    plot_handle.set_ylim([-1, 2])
    
    plot_handle.tick_params(axis='x', labelsize=30)
    plot_handle.tick_params(axis='y', labelsize=30)
    plot_handle.set_xlabel('X(m)',fontsize=30)
    plot_handle.set_ylabel('Y(m)',fontsize=30)

    g = plot_handle.annotate('',(-2,2),**text_kwargs)
    plot_handle.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    """ if id == 1:
        plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], marker='h',markersize=10) # current
    else:
        plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], marker=(8, 0),markersize =10) # current """

    if robotid == 0:
        plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], marker='h', color = 'red',markersize=20)
        plot_handle.plot(goal[0], goal[1], color='red', marker='X',markersize=20)
    elif robotid == 1:
        plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], marker='h',color = 'blue',markersize=20)
        plot_handle.plot(goal[0], goal[1], color='blue', marker='X',markersize=20)
    elif robotid ==2:
        plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], marker=(8,0),color = 'green',markersize=20)
        plot_handle.plot(goal[0], goal[1], color='green', marker='X',markersize=20)
    elif robotid == 3:
        plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], marker=(8,0),color = 'black',markersize=20)
        plot_handle.plot(goal[0], goal[1], color='black', marker='X',markersize=20)





    """ plot_handle.plot(centre_hist[:, 0], centre_hist[:, 1],'k')
    #plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(centre_hist[-1, 0], centre_hist[-1, 1], '8k') """ # current

    
    for i in range(new_obs.T.shape[1]):
        #print('This is obstacle{}'.format(i),new_obs[i,0],new_obs[i,1])
        plot_handle.plot(new_obs[i,0],new_obs[i,1], '8k') # obs

        ell = Ellipse((new_obs[0,0],new_obs[0,1]), 0.89, 0.89, 0)
        ell2 = Ellipse((new_obs[1,0],new_obs[1,1]), 0.89, 0.89, 0)
        ell.set_alpha(0.2)
        ell.set_facecolor(np.array([1, 0, 0]))
        ell2.set_alpha(0.2)
        ell2.set_facecolor(np.array([1, 0, 0]))

    #print((centre_hist))
    ell3 = Ellipse((centre_hist[-1, 0], centre_hist[-1, 1]), 0.6, 0.4, angle_hist[-1])
    #plot_handle.plot(centre_hist[:, 0],centre_hist[:, 1],'2k')
    ell3.set_alpha(0.3)
    ell3.set_facecolor(np.array([1, 0, 1]))
    
    plot_handle.add_artist(ell)
    plot_handle.add_artist(ell2)
    plot_handle.add_artist(ell3)
    #plot_handle.axis('equal')

    #annotate(g,time_str)
    

def annotate(a,x):
    a.set_text(x)