import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math


N = 10
MARKERSIZE = 2

def storing(filename, records):
    for oneline in filename:
        onewords = oneline.split()
        temp = []
        for record in onewords:
            record = float(record)
            temp.append(record)
        records.append(temp)
    records = pd.DataFrame(records)
    records.rename(columns={0:'x', 1:'y', 2:'phi'}, inplace = True)
    return records

if(__name__ == '__main__'):
    f_odometry = open("data//odometry.txt", "r")
    f_biased_trajectory = open("data//biased_trajectory.txt", "r")
    f_estimation = open("data//estimation.txt", "r")
    f_measurement_bias = open("data//measurement_bias.txt", "r")
    f_measurement_true = open("data//measurement_true.txt", "r")
    f_measurement_Kalman = open("data//measurement_Kalman.txt", "r")
    f_MSE = open("data//MSE.txt", "r")
    
    odometry = []; biased_trajectory = []; estimation = []; measurement_bias = []; measurement_true = []; measurement_Kalman = []; MSE = []
    
    odometry = storing(f_odometry, odometry)
    biased_trajectory = storing(f_biased_trajectory, biased_trajectory)
    estimation = storing(f_estimation, estimation)
    measurement_bias = storing(f_measurement_bias, measurement_bias)
    measurement_true = storing(f_measurement_true, measurement_true)
    measurement_Kalman = storing(f_measurement_Kalman, measurement_Kalman)
    
    
    
    flag = 1
    indicator = np.zeros((1, N), dtype = float)
    indicator = indicator[0]

    
    for i in range(N):
        
        for j in range(int(len(estimation) / N)):
            dist = math.sqrt((estimation['x'][i + N*j] - measurement_true['x'][i + N*j])**2 + (estimation['y'][i + N*j] - measurement_true['y'][i + N*j])**2)
            indicator[i] = indicator[i] + dist
    print(indicator/int(len(estimation) / N))
    print(sum(indicator)/int(len(estimation) / N))
    
    
    if(flag):
        for i in range(0, 1):
            plt.figure(figsize=(8, 6), dpi = 80)
            # plt.plot(desired_trajectory['x'], -desired_trajectory['y'], '*', markersize = MARKERSIZE, label = "desired trajectory")
            plt.plot(odometry['x'][i::N], -odometry['y'][i::N], '*', markersize = MARKERSIZE, label = "odometry")
            plt.plot(estimation['x'][i::N], -estimation['y'][i::N], '*', markersize = MARKERSIZE, label = "estimated position")
            plt.plot(measurement_true['x'][i::N], -measurement_true['y'][i::N], '*', markersize = MARKERSIZE, label = "true position")
            # plt.plot(measurement_Kalman['x'][0::N], -measurement_Kalman['y'][0::N], '*', markersize = MARKERSIZE, label = "state estimation after Kalman")
            
            plt.title("robot" + str(i) + " trajectories")
            plt.xlabel('x'); plt.ylabel('y'); 
            plt.legend()
            
            plt.savefig('pics/robot' + str(i) + '.png')
    
    plt.show()
    
    # for oneline in f_MSE:
    #     onewords = oneline.split()
    #     temp = []
    #     for record in onewords:
    #         record = float(record)
    #         temp.append(record)
    #     MSE.append(temp)
    # MSE = pd.DataFrame(MSE)
    # print(MSE)
    # plt.figure(figsize=(8, 6), dpi = 80)
    
    # plt.plot(MSE[1])
    
    # plt.show()
    
    