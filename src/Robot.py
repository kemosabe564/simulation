import numpy as np
import pygame
import math
import random

screen_width = 1080
screen_height = 640

class Robot:
    
    def __init__(self, idx, init_x, init_y, init_phi, endpoint, sensor_r, robot_l, robot_b, data = []):
        self.idx = idx
        self.x = init_x * 1.0
        self.y = init_y * 1.0
        self.phi = init_phi * 1.0
        
        self.timer = 0
        
        self.goalX = endpoint
        
        self.sensor_range = sensor_r * 1.0
        self.l = robot_l # Robot length is 2*l
        self.b = robot_b # Robot breadth is 2*b
        self.data = data
        self.flag = 1
        
        
        self.desired_trajectory = np.array([self.x, self.y, self.phi], float)
        self.biased_trajectory = np.array([self.x, self.y, self.phi], float)
        self.estimation = np.array([self.x, self.y, self.phi], float)
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        self.measurement_bias = np.array([self.x, self.y, self.phi], float)
        self.measurement_Kalman = np.array([self.x, self.y, self.phi], float)
        
                
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
        K1 = 1; K2 = 1
        w1 = 0.5; w2 = 0.5

        v = v0 + K1*random.uniform(-w1*v0, w1*v0) + 0.010
        omega = omega0 + K2*random.uniform(-w2*omega0, w2*omega0) + 0.000
        
        return [v, omega]
        
    def measurement_merge(self, obstacles, robots):
        
        x = []; y = []
        # for i in range(obstacles.num_circ_obsts):
        #     dis = math.sqrt((obstacles.circ_x[i] - self.x)**2 + (obstacles.circ_y[i] - self.y)**2)
        #     if(dis < 80):
        #         x.append(obstacles.circ_x[i])
        #         y.append(obstacles.circ_y[i])
        # print(x)
        i = 0
        for item in robots.distance_table[self.idx]:
            if(item < 80 and item > 0):
                x.append(robots.robots_list[i].x)
                y.append(robots.robots_list[i].y)
            i += 1
            # print(item)

        # print(robots.distance_table[self.idx])
        # print(x)
        if(len(x) > 0):
            idx = random.randint(0, len(x)-1)
            temp = 1#random.randint(0, 1)
            # print(temp)
            if(temp):
                self.measurement_bias[0] = x[idx] + random.uniform(-1, 1)
                self.measurement_bias[1] = y[idx] + random.uniform(-1, 1)

    def measurement_pertub(self, obstacles, robots):
        K1 = 0.1; K2 = 0 
        w1 = 0.1; w2 = 0.1; w3 = 0.1
        self.measurement_bias[0] = self.measurement_true[0] + K1*random.uniform(-w1*self.measurement_true[0], w1*self.measurement_true[0])
        self.measurement_bias[1] = self.measurement_true[1] + K1*random.uniform(-w2*self.measurement_true[1], w2*self.measurement_true[1])
        self.measurement_bias[2] = self.measurement_true[2] + K2*random.uniform(-w3*self.measurement_true[2], w3*self.measurement_true[2])
        
        self.measurement_merge(obstacles, robots)
  
    def update_movement(self, v, omega, v0, omega0, v_biased, omega_biased, obstacles, robots, k_filter):
        
        self.timer += 1
        
        # for the most desired position, without any pertub, either on states or measures
        self.desired_trajectory = self.states_transform(v0, omega0, self.desired_trajectory)
        
        # for the biased trajectory, it uses measurement with bias
        self.biased_trajectory = self.states_transform(v_biased, omega_biased, [self.x, self.y, self.phi])
        
        
        
        # if not stop, states are pertubed
        if(self.flag):
            [v1, omega1] = self.states_perturb(v, omega)
        else:
            [v1, omega1] = [v, omega]
        
        
        # based on the pertubed control input, to update the true states 
        [self.x, self.y, self.phi] = self.states_transform(v1, omega1, [self.x, self.y, self.phi])
        
        # record the true states as measurement_true
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        
        # since the eastimation doesn't know states pertubed, it will update itself based on unbiased control input ans current states
        # self.estimation = self.states_transform(v, omega, [self.x, self.y, self.phi])
        # it will use the last estimation to update the new one
        self.estimation = self.states_transform(v, omega, self.estimation)
        
        
        # for drawing
        self.tip = [self.x + self.l * math.cos(self.phi), self.y + self.l * math.sin(self.phi)]
        self.bottom = [self.x - self.l * math.cos(self.phi), self.y - self.l * math.sin(self.phi)]
        self.bottom_l = [self.bottom[0] - self.b * math.sin(self.phi), self.bottom[1] + self.b * math.cos(self.phi)]
        self.bottom_r = [self.bottom[0] + self.b * math.sin(self.phi), self.bottom[1] - self.b * math.cos(self.phi)]
        
        no_sensor = False
        
        if(no_sensor):
            
            return
        
        # if the robot decide to receive an update from the camera
        if(self.timer % 10 == 0):
            # if not stop, measurements are pertubed, they could merge together, so the camera will give a "fake" data
            if(self.flag):
                self.measurement_pertub(obstacles, robots)
              
            
            K = 1
            if(K):
                print(1)
                # if we decide to use the Kalman filter to correct the fake measurement 
                optimal_state_estimate_k, covariance_estimate_k = k_filter.EKF(self.measurement_bias, self.estimation, [v, omega], 1)
                # obs_vector_z_k = self.measurement_bias, # Most recent sensor measurement
                # state_estimate_k_1 = self.estimation, # Our most recent estimate of the state
                # u_k_1 = [v, omega], # Our most recent control input
                # P_k_1, # Our most recent state covariance matrix
                # dk = 1 # Time interval            
                self.measurement_Kalman = optimal_state_estimate_k
                k_filter.P_k_1 = covariance_estimate_k
                self.estimation = self.measurement_Kalman
            else:
                print(0)
                # if we decide not to use the Kalman filter to correct the fake measurement 
                # then we use the pertubed measurement ans its true 
                self.estimation = self.measurement_bias
                
                    
            
        
        
        
    
        
        
    def go_to_goal(self, endpoints):
        # self.goal = endpoints
        # e = self.data["goalX"] - [self.measurement_Kalman[0], self.measurement_bias[1]]     # error in position
        e = self.goalX - [self.estimation[0], self.estimation[1]]     # error in position
        # e = self.goalX - [self.measurement_bias[0], self.measurement_bias[1]]     # error in position

        e0 = self.goalX - [self.desired_trajectory[0], self.desired_trajectory[1]] 
        e_biased = self.goalX - [self.measurement_bias[0], self.measurement_bias[1]] 
        # K = self.data["vmax"] * (1 - np.exp(- self.data["gtg_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)     # Scaling for velocity
        K = [-0.001, -0.001]
        v = np.linalg.norm(K * e)   # Velocity decreases as bot gets closer to goal
        v0 = np.linalg.norm(K * e0)
        v_biased = np.linalg.norm(K * e_biased)
        
        phi_d = math.atan2(e[1], e[0])  # Desired heading
        phi_d0 = math.atan2(e0[1], e0[0]) 
        phi_biased = math.atan2(e_biased[1], e_biased[0])
         
        omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.estimation[2]), math.cos(phi_d - self.estimation[2]))     # Only P part of a PID controller to give omega as per desired heading
        # omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.measurement_bias[2]), math.cos(phi_d - self.measurement_bias[2]))     # Only P part of a PID controller to give omega as per desired heading
        omega0 = self.data["K_p"]*math.atan2(math.sin(phi_d0 - self.desired_trajectory[2]), math.cos(phi_d0 - self.desired_trajectory[2]))     # Only P part of a PID controller to give omega as per desired heading
        omega_biased = self.data["K_p"]*math.atan2(math.sin(phi_biased - self.measurement_bias[2]), math.cos(phi_biased - self.measurement_bias[2]))
        
        return [v, omega, v0, omega0, v_biased, omega_biased]

    # def avoid_obst(self, obstX):
    #     e = obstX - self.X     # error in position
    #     K = self.data["vmax"] * (1 - np.exp(- self.data["ao_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)      # Scaling for velocity
    #     v = np.linalg.norm(K * e)   # Velocity decreases as bot gets closer to obstacle
    #     phi_d = -math.atan2(e[1], e[0]) # Desired heading
    #     omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.phi), math.cos(phi_d - self.phi))     # Only P part of a PID controller to give omega as per desired heading
    #     return [v, omega]

    def ternimate(self):
        K = 30.0
        distance = math.sqrt((self.goalX[0] - self.x)**2 + (self.goalX[1] - self.y)**2)
        if(distance < K):
            self.flag = 0
            return 0
        else:
            self.flag = 1
            return 1

    def show(self, screen):
        pygame.draw.polygon(screen, (255,0,0), [self.tip, self.bottom_l, self.bottom_r], 0)
        pygame.draw.circle(screen, (255,0,0), (self.x, self.y), 15, 1)
            
            
class Robots:
    
    def __init__(self, n, robots_x, robots_y, robots_phi, goalX, setup):              
        self.number = n
        self.robots_list = []
        self.distance_table = np.empty((self.number, self.number), dtype = float)
        self.distance_table.fill(0)
        
        for i in range(n):
            temp = Robot(i, robots_x[i], robots_y[i], robots_phi[i], goalX[i], 50, 15, 6, setup)
            self.robots_list.append(temp)
            
    def robots_display(self, screen):
        for i in range(self.number):
            self.robots_list[i].show(screen)
            

    def update_distance_table(self):
        # print(self.distance_table)
        for i in range(0, self.number):
            for j in range(i + 1, self.number):
                self.distance_table[i][j] = math.sqrt((self.robots_list[i].x - self.robots_list[j].x)**2 + (self.robots_list[i].y - self.robots_list[j].y)**2)
                self.distance_table[j][i] = self.distance_table[i][j]
    
    def robots_update_movement(self, goalX, obstacle, K_filter):
        
        for i in range(self.number):
            
            if(self.robots_list[i].ternimate() == 1):                          
                [v, omega, v0, omaga0, v_biased, omega_biased] = self.robots_list[i].go_to_goal(goalX[i])
                self.robots_list[i].update_movement(v, omega, v0, omaga0, v_biased, omega_biased, obstacle, self, K_filter)
                # self.robots_list[i].update_movement(0, 0, 0, 0, 0, 0, obstacle, self, K_filter)
            else:      
                self.robots_list[i].update_movement(0, 0, 0, 0, 0, 0, obstacle, self, K_filter)
                # update new GoalX
                goalX[i][0] = random.uniform(0.1*screen_width, 0.9*screen_width)
                goalX[i][1] = random.uniform(0.1*screen_height, 0.9*screen_height)
                while(goalX[i][0] >= screen_width or goalX[i][1] >= screen_height or goalX[i][0] <= 0 or goalX[i][1] <= 0):
                    goalX[i][0] = random.uniform(0.1*screen_width, 0.9*screen_width)
                    goalX[i][1] = random.uniform(0.1*screen_height, 0.9*screen_height)
                
                
                
            
        


