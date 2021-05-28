import numpy as np
from robot import Robot_Sim
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

a = 1
b = 1
safety_dist = 2

def plot_step(ecbf, new_obs, u_hat_acc, state_hist, plot_handle):
    state_hist_plot = np.array(state_hist)
    nom_cont = ecbf.compute_nom()
    multiplier_const = 1
    """ plot_handle.plot([state_hist_plot[-1, 0], state_hist_plot[-1, 0] + multiplier_const *
                u_hat_acc[0]],
                [state_hist_plot[-1, 1], state_hist_plot[-1, 1] + multiplier_const * u_hat_acc[1]], label="Safe")
    plot_handle.plot([state_hist_plot[-1, 0], state_hist_plot[-1, 0] + multiplier_const *
                nom_cont[0]],
                [state_hist_plot[-1, 1], state_hist_plot[-1, 1] + multiplier_const * nom_cont[1]],label="Nominal") """

    plot_handle.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'k')
    plot_handle.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plot_handle.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], '8k') # current
    for i in range(new_obs.shape[1]):
        plot_handle.plot(new_obs[0, i], new_obs[1, i], '8k') # obs
    

    ell = Ellipse((state_hist_plot[-1, 0], state_hist_plot[-1, 1]), a*safety_dist+0.5, b*safety_dist+0.5, 0)
    ell.set_alpha(0.3)
    ell.set_facecolor(np.array([1, 0, 0]))
    
    plot_handle.add_artist(ell)



def main():
    x_init1 = np.array([3, -5])
    goal_init1 =np.array([-6, 4])
    robot1 = Robot_Sim(x_init1, goal_init1, 0)

    ### Robot 2

    x_init2 =np.array([-5, 3])
    goal_init2 =np.array([[4], [-6]])
    Robot2 = Robot_Sim(x_init2, goal_init2,1)

    Robots =[robot1] #,robot2]

    #Define Onstacles

    """ obs1 = np.array([[1],[1]])
    obs2 = np.array([[2.1],[2]])
    obs3 = np.array([[1.5],[1.5]])

    obs = np.hstack((obs1,obs2,obs3)) """

    const_obs = np.array([[-1], [-1]])
    const_obs2 = np.array([[-2], [-0]])

    obs = np.hstack((const_obs2, const_obs))

    #plt.plot([2, 2, 3])

    a, ax1 = plt.subplots()

    for tt in range(20000):
        obstacles = []

        #print(obs)
        #print('Thois is the obstacle shape',obs.shape)

        for robot in Robots:
            obstacles.append(robot.update_obstacles(Robots,obs))

        u_star = []

        for robot in Robots:
            u_star.append(robot.robot_step(np.array(obstacles[robot.id]["obs"])[:, :]))

        if(tt % 50 == 0):
            print(tt)
            plt.cla()
            for robot in Robots:
                
                plot_step(robot.ecbf, np.array(obstacles[robot.id]["obs"])[:, :], u_star[robot.id], robot.state_hist, ax1)

            plt.pause(0.0000000001)


if __name__== '__main__':
    main()