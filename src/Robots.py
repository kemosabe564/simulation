import numpy as np
import math
import random

screen_width = 1080
screen_height = 640

from src.Robot import *



class Robots:
    
    def __init__(self, n, robots_x, robots_y, robots_phi, goalX, setup):              
        self.number = n
        self.robots_list = []
        self.distance_table = np.empty((self.number, self.number), dtype = float)
        self.distance_table.fill(0)
        self.distance_map = np.empty((self.number, 1), dtype = float)
        self.distance_map.fill(0)
        
        for i in range(n):
            temp = Robot(i, robots_x[i], robots_y[i], robots_phi[i], goalX[i], 15, 6, setup)
            self.robots_list.append(temp)
            
    def robots_display(self, screen):
        for i in range(self.number):
            self.robots_list[i].show(screen)
            
    def update_distance_table(self):
        self.distance_map = []
        for i in range(0, self.number):
            temp = []
            for j in range(0, self.number):
                dist = math.sqrt((self.robots_list[i].x - self.robots_list[j].x)**2 + (self.robots_list[i].y - self.robots_list[j].y)**2)
                self.distance_table[i][j] = dist
                self.distance_table[j][i] = self.distance_table[i][j]
                if(dist < 50):
                    temp.append(j)
            idx = random.randint(0, len(temp)-1)
            self.distance_map.append(temp[idx])
        self.distance_map = list(set(self.distance_map))
        # print(self.distance_map)
                  
        
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
            
            if(self.robots_list[i].ternimate() == 1):         
                # 2. robot decide where to go                 
                [v, omega, v0, omaga0, v_biased, omega_biased] = self.robots_list[i].go_to_goal()
                # 3. robot execute the decision
                self.robots_list[i].update_movement(v, omega, v0, omaga0, v_biased, omega_biased)
                # 4. robot receive the information where it is
                self.robots_list[i].update_measurement(v, omega, K_filter, camera)
                
                
            else:
                # if the robot reaches its end, make it stop and change another goal point      
                self.robots_list[i].update_movement(0, 0, 0, 0, 0, 0)
                self.robots_list[i].update_measurement(0, 0, K_filter, camera)
                # update new GoalX
                goalX[i][0] = random.uniform(0.1*screen_width, 0.9*screen_width)
                goalX[i][1] = random.uniform(0.1*screen_height, 0.9*screen_height)
                while(goalX[i][0] >= screen_width or goalX[i][1] >= screen_height or goalX[i][0] <= 0 or goalX[i][1] <= 0):
                    goalX[i][0] = random.uniform(0.1*screen_width, 0.9*screen_width)
                    goalX[i][1] = random.uniform(0.1*screen_height, 0.9*screen_height)

