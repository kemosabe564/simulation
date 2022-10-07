from distutils.core import setup
import numpy as np
import pygame
from pygame.locals import *

import random



# import sys
# sys.path.append('../')
from src.Robot import *
from src.Obstacles import *




if(__name__ == '__main__'):
    
    measurement_true_file = open('data\\measurement_true.txt', 'w')
    measurement_bias_file = open('data\\measurement_bias.txt', 'w')
    estimation_file = open('data\\estimation.txt', 'w')
    
    # init
    screen_width = 640; screen_height = 480
    screen = pygame.display.set_mode([screen_width, screen_height], DOUBLEBUF)
    robot_x = 100; robot_y = 100; robot_phi = 0; robot_l = 15; robot_b = 6  # Initial position
    sensor_r = 50   # Sensor skirt radius
    goalX = np.array([300, 300])    # goal position
    
    obstacle = Obstacles(screen_width, screen_height, 6, 10, 50)
    setup = {"goalX":goalX, "vmax":0.5, "gtg_scaling":0.0001, "K_p":0.01, "ao_scaling":0.00005}
    robot = Robot(robot_x, robot_y, robot_phi, sensor_r, robot_l, robot_b, setup)
        
    
    pygame.init()
    pygame.display.set_caption('Unicycle robot')
    clock = pygame.time.Clock()
    ticks = pygame.time.get_ticks()
    frames = 0
    
    v = 0
    omega = 0
    
    
    while(1):
        measurement_true_file.write(str(format(robot.measurement_true[0], '.6f')) + " " + str(format(robot.measurement_true[1], '.6f')) + " " + str(format(robot.measurement_true[2], '.6f')))
        measurement_true_file.write("\n")
        measurement_bias_file.write((str(format(robot.measurement_bias[0], '.6f')) + " " + str(format(robot.measurement_bias[1], '.6f')) + " " + str(format(robot.measurement_bias[2], '.6f'))))        
        measurement_bias_file.write("\n")
        estimation_file.write((str(format(robot.estimation[0], '.6f')) + " " + str(format(robot.estimation[1], '.6f')) + " " + str(format(robot.estimation[2], '.6f'))))
        estimation_file.write("\n")
        
        
        # print(robot.measurement_true)
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            measurement_true_file.close()
            measurement_bias_file.close()
            estimation_file.close()
            break
        
        screen.fill((50, 55, 60))   # background
        pygame.draw.circle(screen, (0,255,0), goalX, 8, 0)  # Draw goal

        # obstacle.draw_circular_obsts(screen)
        robot.show(screen)
        
        # closest_obstacles = []; closest_dist = max(screen_height, screen_width)
        
        # for i in range(obstacle.num_circ_obsts):
        #     distance = math.sqrt((obstacle.circ_x[i] - robot.x)**2 + (obstacle.circ_y[i] - robot.y)**2)
        #     if(distance <= obstacle.radius[i] + robot.sensor_range):
        #         closest_obstacles = [obstacle.circ_x[i], obstacle.circ_y[i]]
        #         closest_dist = distance
        # # print(len(closest_obstacles))
        # if(len(closest_obstacles) > 1):
        #     [v, omega] = robot.avoid_obst(closest_obstacles)   
        # else:
        #     [v, omega] = robot.go_to_goal()

        if(robot.ternimate() == 1):                          
            [v, omega] = robot.go_to_goal()
            robot.update_movement(v, omega)
        else:      
            robot.update_movement(0, 0)
            
        
        clock.tick(200)     # To limit fps, controls speed of the animation
        fps = (frames*1000)/(pygame.time.get_ticks() - ticks)   # calculate current fps
        
        # Update PyGame display
        pygame.display.flip()
        frames += 1
        