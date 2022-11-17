import numpy as np
import pygame
import math
import random



class Camera:
    
    def __init__(self, n, robots_x, robots_y, robots_phi):
        self.number = n
        self.curr_number = n
        
        self.position_list = np.zeros((n, 3), dtype = float)
        self.measurement_list = np.zeros((n, 3), dtype = float)
        
        for i in range(n):
            self.position_list[i][0] = robots_x[i]; self.measurement_list[i][0] = robots_x[i]
            self.position_list[i][1] = robots_y[i]; self.measurement_list[i][1] = robots_y[i]
            self.position_list[i][2] = robots_phi[i]; self.measurement_list[i][2] = robots_phi[i]
            
        
    def update_measurements(self, idx, true_position):
        self.position_list[idx] = true_position
        
        

        
        
    def measurement_merge_perturb(self, robots):
        self.measurement_list = []
        K1 = 0.05; K2 = 0 
        w1 = 0.1; w2 = 0.1; w3 = 0
        flag = True
        # considering the worst case that if there is somethin
        i = 0
        for item in robots.distance_map:
            
            self.measurement_list.append(self.position_list[item])
            if(flag):
                self.measurement_list[i][0] = self.measurement_list[i][0] + K1*random.uniform(-w1*self.position_list[i][0], w1*self.position_list[i][0])
                self.measurement_list[i][1] = self.measurement_list[i][1] + K1*random.uniform(-w2*self.position_list[i][1], w2*self.position_list[i][1])
                self.measurement_list[i][2] = self.measurement_list[i][2] + K2*random.uniform(-w3*self.position_list[i][2], w3*self.position_list[i][2])
            i += 1        
        # print(self.measurement_list)    
        # self.measurement_perturb()

    # def measurement_perturb(self):
    #     K1 = 0.1; K2 = 0 
    #     w1 = 0.1; w2 = 0.1; w3 = 0
    #     for idx in range(self.number):
    #         self.measurement_list[idx][0] = self.measurement_list[idx][0] + K1*random.uniform(-w1*self.position_list[idx][0], w1*self.position_list[idx][0])
    #         self.measurement_list[idx][1] = self.measurement_list[idx][1] + K1*random.uniform(-w2*self.position_list[idx][1], w2*self.position_list[idx][1])
    #         self.measurement_list[idx][2] = self.measurement_list[idx][2] + K2*random.uniform(-w3*self.position_list[idx][2], w3*self.position_list[idx][2])
                    
        
        # x = []; y = []; i = 0
        
        # for item in robots.distance_table[self.idx]:
        #     if(item < 80 and item > 0):
        #         x.append(robots.robots_list[i].x)
        #         y.append(robots.robots_list[i].y)
        #     i += 1

        # # print(robots.distance_table[self.idx])
        # # print(x)
        # if(len(x) > 0):
        #     idx = random.randint(0, len(x)-1)
        #     temp = 1

        #     if(temp):
        #         self.measurement_bias[0] = x[idx] + random.uniform(-1, 1)
        #         self.measurement_bias[1] = y[idx] + random.uniform(-1, 1)

        
        
        