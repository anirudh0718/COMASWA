import csv
import pandas as pd
import matplotlib.pyplot as plt
import sys


headers = ['h0','h1','h2','h3','h4','h5','h6','h7','h8','h9']
csv_file = pd.read_csv('h111obs_1form.csv')

h0 = csv_file['h0']
h1 = csv_file['h1']
h2 = csv_file['h2']
h3 = csv_file['h3']
h4 = csv_file['h4']
h5 = csv_file['h5']
h6 = csv_file['h6']
h7 = csv_file['h7']
h8 = csv_file['h8']
#h9 = csv_file['h9']

fig, ax1 = plt.subplots()
#csv_file.plot()
x = +1.6

ax1.plot(h0+x,label ='h0')
ax1.plot(h1+x,label ='h1')
#ax1.plot(h2,label ='h2')
#ax1.plot(h3+x,label ='h3')
#ax1.plot(h4+x,label ='h4')
#ax1.plot(h5+x,label ='h5')
#ax1.plot(h6+x,label ='h6')
#ax1.plot(h7+x,label ='h7')
#ax1.plot(h8+x,label ='h8')
#ax1.plot(h9,label ='h9')

plt.legend()
plt.show()
