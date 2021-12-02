import csv
import pandas as pd
import matplotlib

import matplotlib.pyplot as plt
import sys
import numpy as np


csv_file = pd.read_csv('h011obs_1form.csv')

h0 = csv_file['h0']
t = [a/100 for a in range(len(h0))]

h1 = csv_file['h1']
h2 = csv_file['h2']
h3 = csv_file['h3']
h4 = csv_file['h4']
h5 = csv_file['h5']
h6 = csv_file['h6']
h7 = csv_file['h7']
h8 = csv_file['h8']
#h9 = csv_file['h9']

fig, ax1 = plt.subplots(figsize=(10,6))
#csv_file.plot()
x = +0.02
y = 0

rob = '4'

if rob == '1':
    n1,n2,n3 = '2','3','4'
elif rob == '2':
    n1,n2,n3 = '1','3','4'
elif rob == '3':
    n1,n2,n3 = '1','2','4'
elif rob == '4':
    n1,n2,n3 = '2','3','1'

params = {'mathtext.default': 'regular' }          
plt.rcParams.update(params)




#ax1.plot(h9,label ='h9')
i = 1

if i ==1:
    ax1.plot(t,h0+x,label ='$h^{obs1}_{'+rob+'}$')
    ax1.plot(t,h1+x,label ='$h^{obs2}_{'+rob+'}$')
    #ax1.plot(t,h2,label ='$h^{Afc}_{'+rob+'}$')
if i==2:

    ax1.plot(t,h3+x,label ='$h^{formg}_{'+rob+n1+'}$')
    ax1.plot(t,h4,label ='$h^{formg}_{'+rob+n2+'}$')
    ax1.plot(t,h5+x,label ='$h^{formg}_{'+rob+n3+'}$')
    ax1.plot(t,h6,label ='$h^{forml}_{'+rob+n1+'}$')
    ax1.plot(t,h7+x,label ='$h^{forml}_{'+rob+n2+'}$')
    ax1.plot(t,h8+x,label ='$h^{forml}_{'+rob+n3+'}$')
if i==3:
    ax1.plot(t,h3+x,label ='$h^{formg}_{'+rob+n1+'}$')
    ax1.plot(t,h4+x,label ='$h^{forml}_{'+rob+n1+'}$')

plt.legend()
plt.xlabel('time(s)', fontsize=18)
plt.ylabel('safety function', fontsize=16)
plt.show()
