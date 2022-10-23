import matplotlib.pyplot as plt
import pandas as pd

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
    
    plt.plot(desired_trajectory['x'], desired_trajectory['y'], '*', markersize = 1)
    plt.plot(biased_trajectory['x'], biased_trajectory['y'], '*', markersize = 1)
    plt.plot(measurement_true['x'], measurement_true['y'], '*', markersize = 1)
    plt.plot(measurement_bias['x'], measurement_bias['y'], '*', markersize = MARKERSIZE)
    plt.plot(measurement_Kalman['x'], measurement_Kalman['y'], '*', markersize = MARKERSIZE)
    plt.ylabel('some numbers')
    plt.show()