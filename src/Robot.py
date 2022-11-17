import numpy as np
import pygame
import math
import random

screen_width = 1080
screen_height = 640


class Robot:
    
    def __init__(self, idx, init_x, init_y, init_phi, endpoint, robot_l, robot_b, data = []):
        self.timer = 0
        self.idx = idx
        self.x = init_x * 1.0
        self.y = init_y * 1.0
        self.phi = init_phi * 1.0
        self.goalX = endpoint
        # P controllor parameters
        self.data = data
        
        # whether the robot reaches its endpoint
        self.moving = True
        # whether the robot updates its init points
        self.start = False
        
        # buffer
        self.position_buffer = []
        self.camera_buffer = []
        
        
        
        
        # for logging data
        self.desired_trajectory = np.array([self.x, self.y, self.phi], float)
        self.biased_trajectory = np.array([self.x, self.y, self.phi], float)
        
        self.estimation = np.array([self.x, self.y, self.phi], float)
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        self.measurement_bias = np.array([self.x, self.y, self.phi], float)
        
        self.measurement_Kalman = np.array([self.x, self.y, self.phi], float)
        
        
        # for drawing
        self.l = robot_l # Robot length is 2*l
        self.b = robot_b # Robot breadth is 2*b
        temp_sin = math.sin(self.phi)
        temp_cos = math.cos(self.phi)
        self.tip = [self.x + self.l * temp_cos, self.y + self.l * temp_sin]
        self.bottom = [self.x - self.l * temp_cos, self.y - self.l * temp_sin]
        self.bottom_l = [self.bottom[0] - self.b * temp_sin, self.bottom[1] + self.b * temp_cos]
        self.bottom_r = [self.bottom[0] + self.b * temp_sin, self.bottom[1] - self.b * temp_cos]


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

        
    def update_movement(self, v, omega, v0, omega0, v_biased, omega_biased):
        
        self.timer += 1
        
        # for the most desired position, without any pertub, either on states or measures
        self.desired_trajectory = self.states_transform(v0, omega0, self.desired_trajectory)
        
        # for the biased trajectory, it uses measurement with bias
        self.biased_trajectory = self.states_transform(v_biased, omega_biased, [self.x, self.y, self.phi])
              
        # if not stop, states are pertubed
        if(self.moving):
            [v1, omega1] = self.states_perturb(v, omega)
        else:
            [v1, omega1] = [v, omega]
               
        # based on the pertubed control input, to update the true states 
        [self.x, self.y, self.phi] = self.states_transform(v1, omega1, [self.x, self.y, self.phi])
        
        
                
        # for drawing
        temp_sin = math.sin(self.phi)
        temp_cos = math.cos(self.phi)
        self.tip = [self.x + self.l * temp_cos, self.y + self.l * temp_sin]
        self.bottom = [self.x - self.l * temp_cos, self.y - self.l * temp_sin]
        self.bottom_l = [self.bottom[0] - self.b * temp_sin, self.bottom[1] + self.b * temp_cos]
        self.bottom_r = [self.bottom[0] + self.b * temp_sin, self.bottom[1] - self.b * temp_cos]
    
    
    def update_measurement(self, v, omega, k_filter, camera):

        # record the true states as measurement_true
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        
        # since the eastimation doesn't know states pertubed, it will update itself based on unbiased control input ans current states
        # self.estimation = self.states_transform(v, omega, [self.x, self.y, self.phi])
        # it will use the last estimation to update the new one
        self.estimation = self.states_transform(v, omega, self.estimation)
               
        no_sensor = False
        
        if(no_sensor):      
            return
        
        # if the robot decide to receive an update from the camera
        if(self.timer % 10 == 0):
            
            # if not stop, measurements are pertubed, they could merge together, so the camera will give a "fake" data
            # update the measurement
            if(self.moving):
                
                # self.measurement_pertub(robots)
                # self.measurement_bias = camera.measurement_list[self.idx]
                MAX = screen_width + screen_height; i = 0; idx = 0
                for item in camera.measurement_list:
                    dist = math.sqrt((self.x - item[0])**2 + (self.y - item[1])**2)
                    if(dist < MAX):
                        MAX = dist
                        idx = i
                    i += 1
                # print(self.idx, ":")
                # print(camera.measurement_list[idx])
                self.measurement_bias = camera.measurement_list[idx]
                
            K = 1
            if(K):
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
                # if we decide not to use the Kalman filter to correct the fake measurement 
                # then we use the pertubed measurement ans its true 
                self.estimation = self.measurement_bias
                
                                    
    def go_to_goal(self):
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


    def ternimate(self):
        K = 30.0
        distance = math.sqrt((self.goalX[0] - self.x)**2 + (self.goalX[1] - self.y)**2)
        if(distance < K):
            self.moving = 0
            return 0
        else:
            self.moving = 1
            return 1


    def show(self, screen):
        pygame.draw.polygon(screen, (255,0,0), [self.tip, self.bottom_l, self.bottom_r], 0)
        pygame.draw.circle(screen, (255,0,0), (self.x, self.y), 15, 1)
            
            

                
                
            
        


