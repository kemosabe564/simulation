import numpy as np
import pygame
import math
import random



class Camera:
    
    def __init__(self, n, robots_x, robots_y, robots_phi):
        self.number = n
        self.curr_number = n
        
        self.position_list = np.zeros((n, 3), dtype = float)
        self.measurement_list = []#np.zeros((n, 3), dtype = float)
        
        for i in range(n):
            self.position_list[i][0] = robots_x[i]; #self.measurement_list[i][0] = robots_x[i]
            self.position_list[i][1] = robots_y[i]; #self.measurement_list[i][1] = robots_y[i]
            self.position_list[i][2] = robots_phi[i]; #self.measurement_list[i][2] = robots_phi[i]
            
        
    def update_measurements(self, idx, true_position):
        self.position_list[idx] = true_position
                
        
    def measurement_merge_perturb(self, robots):
        self.measurement_list = []
        mu = 0.0; sig_x = 0.005; sig_y = 0.005; sig_phi = 0.0
        
        # K1 = 0.05; K2 = 0 
        # w1 = 0.1; w2 = 0.1; w3 = 0
        # perturb = True
        
        robot_list = np.zeros((1, self.number), dtype = int)
        robot_list = robot_list[0]
        
        
        for i in range(0, len(robots.distance_table)):
            count = 0.0
            sum_x = 0.0
            sum_y = 0.0
            if(robot_list[i]):
                continue
            robot_list[i] = 1
            for j in range(i, len(robots.distance_table[0])):
                
                if(robots.distance_table[i][j] < 50):
                    robot_list[j] = 1
                    count = count + 1
                    sum_x = sum_x + robots.robots_list[j].x
                    sum_y = sum_y + robots.robots_list[j].y
            temp = random.uniform(0, 1)
            if(temp < 0.95):
                self.measurement_list.append([sum_x/count, sum_y/count, robots.robots_list[i].phi])
        
        
        for item in self.measurement_list:
            
            # item[0] = item[0] + K1*random.uniform(-w1*item[0], w1*item[0])
            # item[1] = item[1] + K1*random.uniform(-w2*item[1], w2*item[1])
            # item[2] = item[2] + K2*random.uniform(-w3*item[2], w3*item[2])    
            item[0] = item[0] + np.random.normal(mu, sig_x, 1)[0]
            item[1] = item[1] + np.random.normal(mu, sig_y, 1)[0]
            item[2] = item[2] + np.random.normal(mu, sig_phi, 1)[0]                


        # print(self.measurement_list) 
        # print(len(self.measurement_list))


        
        