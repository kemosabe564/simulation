import numpy as np
import math
import random

screen_width = 1080
screen_height = 640

from src.Robot import *



class Robots:
    
    def __init__(self, n, robots_x, robots_y, robots_phi, omega0, goalX, setup):              
        self.number = n
        self.robots_list = []
        self.distance_table = np.empty((self.number, self.number), dtype = float)
        self.distance_table.fill(0)
        # self.distance_map = np.empty((self.number, 1), dtype = float)
        # self.distance_map.fill(0)
        
        for i in range(n):
            temp = Robot(i, robots_x[i], robots_y[i], robots_phi[i], omega0[i], goalX[i], 15, 6, setup)
            self.robots_list.append(temp)
            
    def robots_display(self, screen):
        for i in range(self.number):
            self.robots_list[i].show(screen)
            
    def update_distance_table(self):
        # self.distance_map = []
        for i in range(0, self.number):
            # temp = []
            for j in range(i, self.number):
                dist = math.sqrt((self.robots_list[i].x - self.robots_list[j].x)**2 + (self.robots_list[i].y - self.robots_list[j].y)**2)
                self.distance_table[i][j] = dist
                self.distance_table[j][i] = self.distance_table[i][j]
                # if(dist < 35):
                #     temp.append(j)
            # idx = random.randint(0, len(temp)-1)
            # self.distance_map.append(temp[idx])
        # print(self.distance_map)
        # self.distance_map = list(set(self.distance_map))
        # print(self.distance_map)
        # print(self.distance_table)             
        
    def robots_simulation_loop(self, goalX, K_filter, camera):
        
        # 1. camera update the measurements and give biased measurement
        # 2. robot decide where to go
        # 3. robot execute the decision
        # 4. robot receive the information where it is  
        
        # 1. camera update the measurements and give biased measurement
        for i in range(self.number):
            camera.update_measurements(i, self.robots_list[i].measurement_true)
        camera.measurement_merge_perturb(self)
            
            
            
        for i in range(self.number):
            self.robots_list[i].robot_loop(goalX[i], K_filter, camera)



