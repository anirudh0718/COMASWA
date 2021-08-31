import numpy as np
import matplotlib.pyplot as plt


h = np.loadtxt('test.out')

plt.figure()
plt.plot(h)
plt.show()