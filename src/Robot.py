import numpy as np
import pygame
import math
import random

from src.setup import *


no_sensor = False

if MODE == "normal":
    no_sensor = True
    sr_KALMAN = 0
    mr_KALMAN = 0
elif MODE == "sr_EKF":
    no_sensor = False
    sr_KALMAN = 1
    mr_KALMAN = 0
elif MODE == "mr_EKF":
    no_sensor = False
    sr_KALMAN = 0
    mr_KALMAN = 1

if EKF_STYLE == "cascade":
    EKF_style = 0
elif EKF_STYLE == "OWA":
    EKF_style = 1
    
class Robot:
    
    def __init__(self, idx, init_x, init_y, init_phi, omega0, endpoint, robot_l, robot_b, data = []):
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
        self.start = True
        
        # buffer
        self.buffer_size = 5
        # self.camera_buffer = [[] for i in range(3)]

        self.odo_error_buffer = [0.1] * self.buffer_size
        self.camera_error_buffer = [0.1] * self.buffer_size
        
        
        # speed 0.1-0.15
        # angle -0.0001 0.0001
        self.x0 = init_x
        self.y0 = init_y
        self.phi0 = init_phi * 1.0
        self.omega0 = omega0
        
        # for logging data
        self.desired_trajectory = np.array([self.x, self.y, self.phi], float)
        self.biased_trajectory = np.array([self.x, self.y, self.phi], float)
        
        
        self.odometry = np.array([self.x, self.y, self.phi0], float)
        self.estimation = np.array([self.x, self.y, self.phi0], float)
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
        mu = 0.0; sig_v = 0.01; sig_omega = 0.0005
        # K1 = 1; K2 = 0.5
        # w1 = 0.5; w2 = 0.5
        # v = v0 + K1*random.uniform(-w1*v0, w1*v0) + 0.000
        # omega = omega0 + K2*random.uniform(-w2*omega0, w2*omega0) + 0.000
        v = v0 + np.random.normal(mu, sig_v, 1)[0]
        omega = omega0 + np.random.normal(mu, sig_omega, 1)[0]
        
        return [v, omega]

        
    def update_movement(self, v, omega):
        
        self.timer += 1
        
        # # for the most desired position, without any pertub, either on states or measures
        # self.desired_trajectory = self.states_transform(v0, omega0, self.desired_trajectory)
        
        # # for the biased trajectory, it uses measurement with bias
        # self.biased_trajectory = self.states_transform(v_biased, omega_biased, [self.x, self.y, self.phi])
              
        # if not stop, states are pertubed
        if(self.moving):
            [v1, omega1] = self.states_perturb(v, omega)
        else:
            [v1, omega1] = [v, omega]
               
        # based on the pertubed control input, to update the true states 
        [self.x, self.y, self.phi] = self.states_transform(v1, omega1, [self.x, self.y, self.phi])
        
        [self.x0, self.y0, self.phi0] = self.states_transform(v1, omega1, [self.x0, self.y0, self.phi0])
        
        self.odometry = self.states_transform(v, omega, self.odometry)
        
                
        # for drawing
        temp_sin = math.sin(self.phi)
        temp_cos = math.cos(self.phi)
        self.tip = [self.x + self.l * temp_cos, self.y + self.l * temp_sin]
        self.bottom = [self.x - self.l * temp_cos, self.y - self.l * temp_sin]
        self.bottom_l = [self.bottom[0] - self.b * temp_sin, self.bottom[1] + self.b * temp_cos]
        self.bottom_r = [self.bottom[0] + self.b * temp_sin, self.bottom[1] - self.b * temp_cos]
    
    # def update_measurement_statr(self, v, omega, camera):
    #     self.measurement_true = np.array([self.x, self.y, self.phi], float)
    #     self.estimation = [self.x0, self.y0, self.phi0]
        
        
    #     if(self.timer % 10 == 0):
            

    #         i = 0
    #         # print(camera.measurement_list)
    #         if(len(camera.measurement_list) == 3):
    #             self.position_buffer.append([self.x0, self.y0])
    #             for item in camera.measurement_list:
            
    #                 self.camera_buffer[i].append([item[0], item[1]])
                    
    #                 i += 1
    #         # print(self.camera_buffer)
 
    #     if(len(self.position_buffer) >= self.buffer_size):
    #         self.determine_camera()
    
    # def determine_camera(self):
    #     # print(self.position_buffer)
   
    #     # print(self.camera_buffer[0])
        
    #     # normalize
    #     for i in range(3):
    #         base_x = self.camera_buffer[i][0][0]
    #         base_y = self.camera_buffer[i][0][1]
    #         for j in range(self.buffer_size-1):
    #             self.camera_buffer[i][j][0] = self.camera_buffer[i][j][0] - base_x
    #             self.camera_buffer[i][j][1] = self.camera_buffer[i][j][1] - base_y
    #         # for item in self.camera_buffer[i]:
    #         #     item[0] = item[0] - base_x
    #         #     item[1] = item[1] - base_y
    #     # print(self.camera_buffer[1])        
            
    #     abs_value = []
    #     min_dist = 10 * screen_height
    #     min_idx = 0
    #     for i in range(3):
    #         temp = 0
    #         for j in range(len(self.position_buffer)):
    #             dist = math.sqrt((self.position_buffer[j][0] - self.camera_buffer[i][j][0])**2 + (self.position_buffer[j][1] - self.camera_buffer[i][j][1])**2)
    #             temp += dist
            
    #         if(temp <= min_dist):
    #             min_dist = temp
    #             min_idx = i
    #         abs_value.append(temp)
        
    #     print(abs_value)
    #     print(min_idx)
        
            
    #     self.estimation[0] = self.camera_buffer[min_idx][self.buffer_size-1][0]
    #     self.estimation[1] = self.camera_buffer[min_idx][self.buffer_size-1][1]
    #     # print(self.camera_buffer[0])
    #     self.start = True
    
    def update_measurement_cascade(self, v, omega, k_filter_camera, k_filter_odo, camera):

        # record the true states as measurement_true
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        
        # since the eastimation doesn't know states pertubed, it will update itself based on unbiased control input ans current states
        # self.estimation = self.states_transform(v, omega, [self.x, self.y, self.phi])
        # it will use the last estimation to update the new one
        
        
        self.estimation = self.states_transform(v, omega, self.estimation)

        
        if(no_sensor):      
            return
        
        # 
        k_filter_odo.R_k = np.array([[1.0,   0,    0],
                                     [  0, 1.0,    0],
                                     [  0,    0, 1.0]]) 
        k_filter_odo.Q_k = np.array([[0.01,   0,    0],
                                     [  0, 0.01,    0],
                                     [  0,    0, 0.005]]) 
        optimal_state_estimate_k, covariance_estimate_k = k_filter_odo.sr_EKF(self.odometry, self.estimation, [v, omega], 1)
        # obs_vector_z_k = self.measurement_bias, # Most recent sensor measurement
        # state_estimate_k_1 = self.estimation, # Our most recent estimate of the state
        # u_k_1 = [v, omega], # Our most recent control input
        # P_k_1, # Our most recent state covariance matrix
        # dk = 1 # Time interval            
        self.measurement_Kalman = optimal_state_estimate_k
        k_filter_odo.P_k_1 = covariance_estimate_k
        self.estimation = self.measurement_Kalman
        
        
        # the camera information is coming        
        if(self.timer % 5 == 0):
            
            # if not stop, measurements are pertubed, they could merge together, so the camera will give a "fake" data
            # update the measurement
            if(self.moving):
                
                # self.measurement_pertub(robots)
                # self.measurement_bias = camera.measurement_list[self.idx]
                MAX_dist = 10 * screen_width + 10 * screen_height; i = 0; idx = 0
                for item in camera.measurement_list:
                    dist = math.sqrt((self.estimation[0] - item[0])**2 + (self.estimation[1] - item[1])**2)
                    if(dist < MAX_dist):
                        MAX_dist = dist
                        idx = i
                    i += 1
                # print(self.idx, ":")
                # print(camera.measurement_list[idx])
                if(MAX_dist > 30):
                    return
                
                # position (x, y)
                self.measurement_bias = camera.measurement_list[idx]
                # self.measurement_bias = camera.measurement_list[self.idx]
                
                self.measurement_bias[2] = self.estimation[2]
           
            
                
            if(sr_KALMAN and ~mr_KALMAN):
                # if we decide to use the Kalman filter to correct the fake measurement 
                # dist = math.sqrt((self.estimation[0] - self.measurement_bias[0])**2 + (self.estimation[1] - self.measurement_bias[1])**2)
                # print(len(camera.measurement_list))
                
                optimal_state_estimate_k, covariance_estimate_k = k_filter_camera.sr_EKF(self.measurement_bias, self.estimation, [v, omega], 1)
                # obs_vector_z_k = self.measurement_bias, # Most recent sensor measurement
                # state_estimate_k_1 = self.estimation, # Our most recent estimate of the state
                # u_k_1 = [v, omega], # Our most recent control input
                # P_k_1, # Our most recent state covariance matrix
                # dk = 1 # Time interval            
                self.measurement_Kalman = optimal_state_estimate_k
                k_filter_camera.P_k_1 = covariance_estimate_k
                self.estimation = self.measurement_Kalman
            elif(mr_KALMAN and ~sr_KALMAN):
                
                optimal_state_estimate_k, covariance_estimate_k = k_filter_camera.mr_EKF(self.measurement_bias, self.estimation, [v, omega], 1)
                self.measurement_Kalman = optimal_state_estimate_k
                k_filter_camera.P_k_1 = covariance_estimate_k
                self.estimation = self.measurement_Kalman
                
            else:
                # if we decide not to use the Kalman filter to correct the fake measurement 
                # then we use the pertubed measurement ans its true 
                self.estimation = self.measurement_bias
                
     
    # def random_moving(self, omega):
        v = random.uniform(0.10, 0.20)
        if(self.timer % 5 == 0):
            omega += random.uniform(-0.000, 0.000)
        return [v, omega] 
    
    def update_measurement_OWA(self, v, omega, k_filter_camera, k_filter_odo, camera):
        # record the true states as measurement_true
        self.measurement_true = np.array([self.x, self.y, self.phi], float)
        
        # since the eastimation doesn't know states pertubed, it will update itself based on unbiased control input ans current states
        # self.estimation = self.states_transform(v, omega, [self.x, self.y, self.phi])
        # it will use the last estimation to update the new one
        
        
        self.estimation = self.states_transform(v, omega, self.estimation)

        
        if(no_sensor):      
            return
        
        # 
        k_filter_odo.R_k = np.array([[1.0,   0,    0],
                                     [  0, 1.0,    0],
                                     [  0,    0, 1.0]]) 
        k_filter_odo.Q_k = np.array([[0.0001,   0,    0],
                                     [  0, 0.0001,    0],
                                     [  0,    0, 0.0001]]) 
        optimal_state_estimate_k, covariance_estimate_k = k_filter_odo.sr_EKF(self.odometry, self.estimation, [v, omega], 1)
        # obs_vector_z_k = self.measurement_bias, # Most recent sensor measurement
        # state_estimate_k_1 = self.estimation, # Our most recent estimate of the state
        # u_k_1 = [v, omega], # Our most recent control input
        # P_k_1, # Our most recent state covariance matrix
        # dk = 1 # Time interval            
        self.measurement_Kalman = optimal_state_estimate_k
        
        k_filter_odo.P_k_1 = covariance_estimate_k
        
        estimation_odo = self.measurement_Kalman
        
        
        
        # the camera information is coming        
        if(self.timer % 5 == 0):
            
            
            
            
            # if not stop, measurements are pertubed, they could merge together, so the camera will give a "fake" data
            # update the measurement
            if(self.moving):
                
                # self.measurement_pertub(robots)
                # self.measurement_bias = camera.measurement_list[self.idx]
                MAX_dist = 10 * screen_width + 10 * screen_height; i = 0; idx = 0
                for item in camera.measurement_list:
                    dist = math.sqrt((self.estimation[0] - item[0])**2 + (self.estimation[1] - item[1])**2)
                    if(dist < MAX_dist):
                        MAX_dist = dist
                        idx = i
                    i += 1
                # print(self.idx, ":")
                # print(camera.measurement_list[idx])
                if(MAX_dist > 30):
                    return
                
                # position (x, y)
                self.measurement_bias = camera.measurement_list[idx]
                # self.measurement_bias = camera.measurement_list[self.idx]
                
                self.measurement_bias[2] = self.estimation[2]
                
                # TODO:
                # 但是如何跟MAX_dist > 30结合来判断
                # add dist error
                # self.odometry - self.estimation ** 2
                # self.measurement_bias - self.estimation ** 2
                # 
                err_camera = ((self.estimation[0] - self.measurement_bias[0])**2 + (self.estimation[1] - self.measurement_bias[1])**2)
                err_odo = ((self.estimation[0] - self.odometry[0])**2 + (self.estimation[1] - self.odometry[1])**2)
                self.camera_error_buffer.append(err_camera)
                self.camera_error_buffer.pop(0)

                
                self.odo_error_buffer.append(err_odo)
                self.odo_error_buffer.pop(0)
                
                

                
            if(sr_KALMAN and ~mr_KALMAN):
                # if we decide to use the Kalman filter to correct the fake measurement 
                # dist = math.sqrt((self.estimation[0] - self.measurement_bias[0])**2 + (self.estimation[1] - self.measurement_bias[1])**2)
                # print(len(camera.measurement_list))
                
                optimal_state_estimate_k, covariance_estimate_k = k_filter_camera.sr_EKF(self.measurement_bias, self.estimation, [v, omega], 1)
                # obs_vector_z_k = self.measurement_bias, # Most recent sensor measurement
                # state_estimate_k_1 = self.estimation, # Our most recent estimate of the state
                # u_k_1 = [v, omega], # Our most recent control input
                # P_k_1, # Our most recent state covariance matrix
                # dk = 1 # Time interval            
                self.measurement_Kalman = optimal_state_estimate_k
                k_filter_camera.P_k_1 = covariance_estimate_k
                estimation_camera = self.measurement_Kalman
                
            elif(mr_KALMAN and ~sr_KALMAN):
                
                optimal_state_estimate_k, covariance_estimate_k = k_filter_camera.mr_EKF(self.measurement_bias, self.estimation, [v, omega], 1)
                self.measurement_Kalman = optimal_state_estimate_k
                k_filter_camera.P_k_1 = covariance_estimate_k
                estimation_camera = self.measurement_Kalman
                
            else:
                # if we decide not to use the Kalman filter to correct the fake measurement 
                # then we use the pertubed measurement ans its true 
                estimation_camera = self.measurement_bias
                                    
            sum_camera = sum(self.camera_error_buffer)
            sum_odo = sum(self.odo_error_buffer)
            
            
                
            w1 = sum_camera / (sum_camera + sum_odo)
            w2 = sum_odo / (sum_camera + sum_odo)
            
            if(sum_camera > 30 * self.buffer_size):
                w1 = 0.9
                w2 = 0.1
            
            # w1 = 0.2
            # w2 = 0.8
            # if(self.idx == 1):
                # print("odo:", self.odo_error_buffer)
            #     print(sum_odo)
                # print("camera: ", self.camera_error_buffer)
            #     print(sum_camera)
            #     print([estimation_camera, estimation_odo])
                # print([w1, w2])
            self.estimation = w2 * estimation_camera + w1 * estimation_odo
                                    
    def go_to_goal(self):
        # e = self.data["goalX"] - [self.measurement_Kalman[0], self.measurement_bias[1]]     # error in position
        e = self.goalX - [self.estimation[0], self.estimation[1]]     # error in position

        # K_P = self.data["vmax"] * (1 - np.exp(- self.data["gtg_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)     # Scaling for velocity
        K_P = [-0.001, -0.001]
        v = np.linalg.norm(K_P * e)   # Velocity decreases as bot gets closer to goal

        
        phi_d = math.atan2(e[1], e[0])  # Desired heading

         
        omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.estimation[2]), math.cos(phi_d - self.estimation[2]))     # Only P part of a PID controller to give omega as per desired heading
        omega_MAX = 0.5 * PI
        if(omega > omega_MAX):
            omega = omega_MAX
        elif(omega < -omega_MAX):
            omega = -omega_MAX
        
        return [v, omega]


    def ternimate(self):
        MAX_length = 30.0
        distance = math.sqrt((self.goalX[0] - self.x)**2 + (self.goalX[1] - self.y)**2)
        if(distance < MAX_length):
            self.moving = 0
            return 0
        else:
            self.moving = 1
            return 1


    def show(self, screen):
        pygame.draw.polygon(screen, (255,0,0), [self.tip, self.bottom_l, self.bottom_r], 0)
        pygame.draw.circle(screen, (255,0,0), (self.x, self.y), 15, 1)
        # font_obj = pygame.font.Font('freesansbold.ttf', 20)
        # text_surface_obj = font_obj.render(str(self.idx), True, (0, 255, 0), (0, 0, 180))
        # text_rect_obj = text_surface_obj.get_rect()
        # text_rect_obj.center = (self.x, self.y)
        # screen.blit(text_surface_obj, text_rect_obj)


    # 1. camera update the measurements and give biased measurement
    
    def robot_loop(self, goalX, K_filter_camera, k_filter_odo, camera):
        
        if(self.start):
            if(self.ternimate() == 1):         
                    # 2. robot decide where to go                 
                    [v, omega] = self.go_to_goal()
                    # 3. robot execute the decision
                    self.update_movement(v, omega)
                    # 4. robot receive the information where it is
                    if EKF_STYLE == "cascade":
                        self.update_measurement_cascade(v, omega, K_filter_camera, k_filter_odo, camera)
                    elif EKF_STYLE == "OWA":
                        self.update_measurement_OWA(v, omega, K_filter_camera, k_filter_odo, camera)
                    # print(v, omega)
                    
            else:
                # if the robot reaches its end, make it stop and change another goal point      
                self.update_movement(0, 0)
                if EKF_STYLE == "cascade":
                    self.update_measurement_cascade(0, 0, K_filter_camera, k_filter_odo, camera)
                elif EKF_STYLE == "OWA":
                    self.update_measurement_OWA(0, 0, K_filter_camera, k_filter_odo, camera)
                # update new GoalX
                goalX[0] = random.uniform(0.1*screen_width, 0.9*screen_width)
                goalX[1] = random.uniform(0.1*screen_height, 0.9*screen_height)
                while(goalX[0] >= screen_width or goalX[1] >= screen_height or goalX[0] <= 0 or goalX[1] <= 0):
                    goalX[0] = random.uniform(0.1*screen_width, 0.9*screen_width)
                    goalX[1] = random.uniform(0.1*screen_height, 0.9*screen_height)
        else:
            # [v, omega] = self.random_moving(0)
            print(self.omega0)
            print(self.phi0)
            # # [v, omega, v0, omaga0, v_biased, omega_biased] = self.go_to_goal()
            # self.update_movement(v, omega)
            # self.update_measurement_statr(v, omega, camera)
            
            
            # if(self.timer == 500):
            #     self.start = True
            

                
                
            
        


