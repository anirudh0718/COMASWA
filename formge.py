
import numpy as np

l = 2
def gen2(midpoint):

    r1 = np.array([midpoint[0],midpoint[1] + l/2])
    r2 = np.array([midpoint[0],midpoint[1] -l/2])

    return r1,r2