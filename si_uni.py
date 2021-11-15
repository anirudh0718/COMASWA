import numpy as np 

l = 0.1
def create_si_to_uni_dynamics(linear_velocity_gain=2, angular_velocity_limit=np.pi):
   

    def si_to_uni_dyn(dxi, poses):
       
        M,N = np.shape(dxi)

        #print('DXI : ',dxi)
        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain*(a*dxi[0, :] + b*dxi[1, :])

        #dxu[1,:] = ((-b*dxi[0, :] + a*dxi[1, :])*(1/l))
        #dxu[1, :] = angular_velocity_limit*(1/l)*np.arctan2(-b*dxi[0, :] + a*dxi[1, :], dxu[0, :])/(np.pi/2)
        dxu[1, :] = (1/l)*(-b*dxi[0, :] + a*dxi[1, :])
        dxu[1,dxu[1,:]>angular_velocity_limit] = angular_velocity_limit
        dxu[1,dxu[1,:]<-angular_velocity_limit] = -angular_velocity_limit 
        #print('DXU: ',dxu)
        return dxu

    return si_to_uni_dyn