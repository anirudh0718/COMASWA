#!/usr/bin/env python
import numpy as np
from robot import Robot_Sim
from plotrob import plot_step
import  matplotlib.pyplot as plt
from FC_to_Goal import get_rob_gposes,get_pose,get_rob_rec_pos,get_turned_rec,get_turn_orient
from math import degrees
import time
import multiprocessing



import multiprocessing
st = time.perf_counter()
def worker(number):
    print ('Sleeping 1Second')
    time.sleep(1)
    print('Done sleeping')

if __name__ == '__main__':
    

    test = multiprocessing.Process(target=worker, args=([0,1,2,3,4],))
    test2 = multiprocessing.Process(target=worker, args=([0,11,22,33,44],))
    test.start()
    test2.start()
     
    test.join()
    test2.join()
    fin = time.perf_counter()
    print(f'Done {round(fin-st)} seconds')
    
