import matplotlib.pyplot as plt
import pandas as pd


N = 3
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
    f_desired_trajectory = open("data//desired_trajectory.txt", "r")
    f_biased_trajectory = open("data//biased_trajectory.txt", "r")
    f_estimation = open("data//estimation.txt", "r")
    f_measurement_bias = open("data//measurement_bias.txt", "r")
    f_measurement_true = open("data//measurement_true.txt", "r")
    f_measurement_Kalman = open("data//measurement_Kalman.txt", "r")
    desired_trajectory = []; biased_trajectory = []; estimation = []; measurement_bias = []; measurement_true = []; measurement_Kalman = []
    
    desired_trajectory = storing(f_desired_trajectory, desired_trajectory)
    biased_trajectory = storing(f_biased_trajectory, biased_trajectory)
    estimation = storing(f_estimation, estimation)
    measurement_bias = storing(f_measurement_bias, measurement_bias)
    measurement_true = storing(f_measurement_true, measurement_true)
    measurement_Kalman = storing(f_measurement_Kalman, measurement_Kalman)
    
    plt.figure(figsize=(8, 6), dpi = 80)
    
    flag = 1
    
    if(flag):
        # for i in range(0, 1):
        i = 1
        # plt.plot(desired_trajectory['x'], -desired_trajectory['y'], '*', markersize = MARKERSIZE, label = "desired trajectory")
        # plt.plot(biased_trajectory['x'], -biased_trajectory['y'], '*', markersize = MARKERSIZE, label = "biased trajectory")
        plt.plot(estimation['x'][i::3]+1, -estimation['y'][i::3], '*', markersize = MARKERSIZE, label = "estimated position")
        plt.plot(measurement_bias['x'][i::3], -measurement_bias['y'][i::3], '*', markersize = MARKERSIZE, label = "biased measurement")        
        plt.plot(measurement_true['x'][i::3], -measurement_true['y'][i::3], '*', markersize = MARKERSIZE, label = "true position")
        # plt.plot(measurement_Kalman['x'][0::3], -measurement_Kalman['y'][0::3], '*', markersize = MARKERSIZE, label = "state estimation after Kalman")
        
        plt.title("trajectories with odometry only")
        
    else:
        plt.plot(desired_trajectory['x'], -desired_trajectory['y'], '*', markersize = MARKERSIZE, label = "desired trajectory")
        # plt.plot(biased_trajectory['x'], -biased_trajectory['y'], '*', markersize = MARKERSIZE, label = "biased trajectory")
        plt.plot(measurement_true['x'], -measurement_true['y'], '*', markersize = MARKERSIZE, label = "true measurement")
        plt.plot(measurement_bias['x'], -measurement_bias['y'], '*', markersize = MARKERSIZE, label = "biased measurement")
        # plt.plot(measurement_Kalman['x'], -measurement_Kalman['y'], '*', markersize = MARKERSIZE, label = "state estimation after Kalman")
        plt.title("trajectories without Kalman")
        
    plt.xlabel('x'); plt.ylabel('y'); 
    plt.legend()
    plt.show()