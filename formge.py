
import numpy as np

#l = 1
def gen2(midpoint,l):
    
    r1 = np.array([midpoint[0],midpoint[1] + l/2])
    r2 = np.array([midpoint[0],midpoint[1] -l/2])

    return r1,r2


#print(gen2(np.array([1.6,2]),0.5))