#safe_d = np.zeros((2,))
            safe_d = np.array(Robots[i].ecbf.compute_safe(obs,Robots,i,2))

            safe_dxi[:,i] = safe_d.reshape(2,)




print('These are the starting postions of the robots',start)
print('These are goal position f the robots',goal)

exit()

print('This is robot {}s goal'.format(i),Robots[i].goal)


np.array([
    [0, Df_b,ddiag_rec,Df_l],
    [Df_b, 0, Df_l, 0],
    [ddiag_rec, Df_l, 0, Df_b],
    [Df_l, 0,Df_b, 0]
])


ddiag_rec =np.sqrt(np.power(Df_l,2) + np.power(Df_b,2))