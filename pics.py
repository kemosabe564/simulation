# from distutils.core import setup
import numpy as np
# import pygame
# from pygame.locals import *
import matplotlib.pyplot as plt
# import random
import pandas as pd



# import sys
# sys.path.append('../')
# from src.Robot import *
# from src.Obstacles import *


if(__name__ == '__main__'):
    f_estimation = open("data//estimation.txt", "r")
    f_measurement_bias = open("data//measurement_bias.txt", "r")
    f_measurement_true = open("data//measurement_true.txt", "r")
    estimation = []; measurement_bias = []; measurement_true = []
    for oneline in f_estimation:
        onewords = oneline.split()
        temp = []
        for record in onewords:
            record = float(record)
            temp.append(record)
        estimation.append(temp)
        
    for oneline in f_measurement_bias:
        onewords = oneline.split()
        temp = []
        for record in onewords:
            record = float(record)
            temp.append(record)
        measurement_bias.append(temp)
        
    for oneline in f_measurement_true:
        onewords = oneline.split()
        temp = []
        for record in onewords:
            record = float(record)
            temp.append(record)
        measurement_true.append(temp)
        
    
    estimation = pd.DataFrame(estimation)
    measurement_bias = pd.DataFrame(measurement_bias)
    measurement_true = pd.DataFrame(measurement_true)

    estimation.rename(columns={0:'x', 1:'y', 2:'phi'}, inplace = True)
    measurement_bias.rename(columns={0:'x', 1:'y', 2:'phi'}, inplace = True)
    measurement_true.rename(columns={0:'x', 1:'y', 2:'phi'}, inplace = True)
    
    # print(estimation['x'])
    # print(measurement_bias['x'])
    plt.plot(measurement_true['x'], measurement_true['y'], '*')
    plt.plot(measurement_bias['x'], measurement_bias['y'], '*')
    plt.ylabel('some numbers')
    plt.show()