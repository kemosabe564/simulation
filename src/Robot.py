from operator import truediv
from sys import flags
import numpy as np
import pygame
import math
import random

class Robot:
    
    def __init__(self, init_x, init_y, init_phi, sensor_r, robot_l, robot_b, data = []):
        self.x = init_x * 1.0
        self.y = init_y * 1.0
        self.phi = init_phi * 1.0
        self.sensor_range = sensor_r * 1.0
        self.l = robot_l # Robot length is 2*l
        self.b = robot_b # Robot breadth is 2*b
        self.data = data
        self.flag = 1
        
        self.measurement_bias = np.array([self.x, self.y, self.phi], float)
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        self.estimation = np.array([self.x, self.y, self.phi], float)
        
        self.X = np.array([self.x, self.y], float)   
        
             
        self.tip = [self.x + self.l * math.cos(self.phi), self.y + self.l * math.sin(self.phi)]
        self.bottom = [self.x - self.l * math.cos(self.phi), self.y - self.l * math.sin(self.phi)]
        self.bottom_l = [self.bottom[0] - self.b * math.sin(self.phi), self.bottom[1] + self.b * math.cos(self.phi)]
        self.bottom_r = [self.bottom[0] + self.b * math.sin(self.phi), self.bottom[1] - self.b * math.cos(self.phi)]

    def states_transform(self, v = 0, omega = 0, X = [0, 0, 0]):
        X[0] += v*math.cos(X[2])
        X[1] += v*math.sin(X[2])
        X[2] += omega 
        return X
    
    def states_perturb(self, v0 = 0.0, omega0 = 0.0):
        K1 = 0.05; K2 = 0.01
        w1 = 0.1; w2 = 0.1

        v = v0 + K1*random.uniform(-w1*v0, w1*v0)
        omega = omega0 + K2*random.uniform(-w2*omega0, w2*omega0)
        
        return [v, omega]
        

    def measurement_merge(self, obstacles):
        
        x = []; y = []
        for i in range(obstacles.num_circ_obsts):
            dis = math.sqrt((obstacles.circ_x[i] - self.x)**2 + (obstacles.circ_y[i] - self.y)**2)
            if(dis < 75):
                x.append(obstacles.circ_x[i])
                y.append(obstacles.circ_y[i])
        print(x)
        if(len(x) > 0):
            
            idx = random.randint(0, len(x)-1)
            # self.measurement_bias[0] = (self.measurement_bias[0] + x[idx])/2
            # self.measurement_bias[1] = (self.measurement_bias[1] + y[idx])/2
            self.measurement_bias[0] = x[idx]
            self.measurement_bias[1] = y[idx]
        

    def measurement_pertub(self, obstacles):
        K1 = 0.1; K2 = 0.01
        w1 = 0.1; w2 = 0.1; w3 = 0.1
        self.measurement_bias[0] = self.measurement_true[0] + K1*random.uniform(-w1*self.measurement_true[0], w1*self.measurement_true[0])
        self.measurement_bias[1] = self.measurement_true[1] + K1*random.uniform(-w2*self.measurement_true[1], w2*self.measurement_true[1])
        self.measurement_bias[2] = self.measurement_true[2] + K2*random.uniform(-w3*self.measurement_true[2], w3*self.measurement_true[2])
        
        self.measurement_merge(obstacles)

    
    def update_movement(self, v, omega, obstacles):
        
        if(self.flag):
            [v1, omega1] = self.states_perturb(v, omega)
        else:
            [v1, omega1] = [v, omega]
        # self.x += v*math.cos(self.phi)
        # self.y += v*math.sin(self.phi)
        # self.phi += omega

        [self.x, self.y, self.phi] = self.states_transform(v1, omega1, [self.x, self.y, self.phi])
        
        self.estimation = self.states_transform(v, omega, self.estimation)
        
        # self.estimation[0] += v*math.cos(self.estimation[2])
        # self.estimation[1] += v*math.sin(self.estimation[2]) 
        # self.estimation[2] += omega
        self.X = np.array([self.x, self.y], float) 
        
        
        self.measurement_true = np.array([self.x, self.y, self.phi], float) 
        
        
        if(self.flag):
            self.measurement_pertub(obstacles)
        # print(self.measurement_true)
        
        # for drawing
        self.tip = [self.x + self.l * math.cos(self.phi), self.y + self.l * math.sin(self.phi)]
        self.bottom = [self.x - self.l * math.cos(self.phi), self.y - self.l * math.sin(self.phi)]
        self.bottom_l = [self.bottom[0] - self.b * math.sin(self.phi), self.bottom[1] + self.b * math.cos(self.phi)]
        self.bottom_r = [self.bottom[0] + self.b * math.sin(self.phi), self.bottom[1] - self.b * math.cos(self.phi)]
    
        
        
    def go_to_goal(self):
        e = self.data["goalX"] - [self.measurement_bias[0], self.measurement_bias[1]]     # error in position
        K = self.data["vmax"] * (1 - np.exp(- self.data["gtg_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)     # Scaling for velocity
        # K = [0.011, 0.1]
        v = np.linalg.norm(K * e)   # Velocity decreases as bot gets closer to goal
        phi_d = math.atan2(e[1], e[0])  # Desired heading
        omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.phi), math.cos(phi_d - self.phi))     # Only P part of a PID controller to give omega as per desired heading
        return [v, omega]

    # def avoid_obst(self, obstX):
    #     e = obstX - self.X     # error in position
    #     K = self.data["vmax"] * (1 - np.exp(- self.data["ao_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)      # Scaling for velocity
    #     v = np.linalg.norm(K * e)   # Velocity decreases as bot gets closer to obstacle
    #     phi_d = -math.atan2(e[1], e[0]) # Desired heading
    #     omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.phi), math.cos(phi_d - self.phi))     # Only P part of a PID controller to give omega as per desired heading
    #     return [v, omega]

    def ternimate(self):
        K = 25.0
        distance = math.sqrt((self.data["goalX"][0] - self.x)**2 + (self.data["goalX"][1] - self.y)**2)
        if(distance < K):
            self.flag = 0
            return 0
        else:
            self.flag = 1
            return 1

    def show(self, screen):
        pygame.draw.polygon(screen, (255,0,0), [self.tip, self.bottom_l, self.bottom_r], 0)
        pygame.draw.circle(screen, (255,0,0), (self.x, self.y), 15, 1)
            
            
            





