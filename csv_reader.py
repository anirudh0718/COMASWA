import csv
import pandas as pd
import matplotlib.pyplot as plt
from FC_to_Goal import get_turn_orient_ros
import numpy as np
""" c  = pd.read_csv('red.csv')
print(c) """

""" with open('h00.csv','r') as file:
     """
""" colors = 12
headers = ['h0','h1','h2','h3','h4','h5','h6','h7','h8','h9']
csv_file = pd.read_csv('h00.csv')

csv_file.plot()

plt.show()
#print((csv_file['h0']))
l = csv_file['h0'].to_list()
print(len(l)) """

print(get_turn_orient_ros(np.array([1.4,-0.7]),1,0.5 ))