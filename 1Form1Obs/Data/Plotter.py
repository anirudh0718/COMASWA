import csv
import pandas as pd
import matplotlib.pyplot as plt
import sys

sys.path.append(r'/home/localadmin/Comas_new/1FormObs/Data')
headers = ['h0','h1','h2','h3','h4','h5','h6','h7','h8','h9']
csv_file = pd.read_csv('h011obs_1form.csv')

csv_file.plot()

plt.show()
