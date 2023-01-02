from distutils.core import setup
import numpy as np
import pygame
from pygame.locals import *

from src.Robots import *
from src.Kalman import *
from src.Camera import *

from src.setup import *

if(__name__ == '__main__'):
    
    # open the file    
    measurement_Kalman_file = open('data\\measurement_Kalman.txt', 'w')
    measurement_true_file = open('data\\measurement_true.txt', 'w')
    measurement_bias_file = open('data\\measurement_bias.txt', 'w')
    estimation_file = open('data\\estimation.txt', 'w')
    odometry_file = open('data\\odometry.txt', 'w')
    biased_trajectory_file = open('data\\biased_trajectory.txt', 'w')
    file_list = [measurement_Kalman_file, measurement_true_file, measurement_bias_file, estimation_file, odometry_file, biased_trajectory_file]
    
    # init
    screen = pygame.display.set_mode([screen_width, screen_height], DOUBLEBUF)
    
    robots = Robots(robots_num, robots_x, robots_y, robots_phi, omega0, goalX, setup)
    
    K_filter = Kalman()    
    
    camera = Camera(robots_num, robots_x, robots_y, robots_phi)
    
    np.random.seed(100)
    
    SEED = np.random.randint(1000000, size=(1, robots_num))
    SEED = SEED[0]
    
    pygame.init()
    pygame.display.set_caption('Unicycle robot')
    clock = pygame.time.Clock()
    ticks = pygame.time.get_ticks()
    frames = 0
    
    # while loop
    while(1):
        
        for robot in robots.robots_list:
            measurement_Kalman_file.write((str(format(robot.measurement_Kalman[0], '.6f')) + " " + str(format(robot.measurement_Kalman[1], '.6f')) + " " + str(format(robot.measurement_Kalman[2], '.6f'))))
            measurement_Kalman_file.write("\n")
            measurement_true_file.write(str(format(robot.measurement_true[0], '.6f')) + " " + str(format(robot.measurement_true[1], '.6f')) + " " + str(format(robot.measurement_true[2], '.6f')))
            measurement_true_file.write("\n")
            measurement_bias_file.write((str(format(robot.measurement_bias[0], '.6f')) + " " + str(format(robot.measurement_bias[1], '.6f')) + " " + str(format(robot.measurement_bias[2], '.6f'))))        
            measurement_bias_file.write("\n")
            estimation_file.write((str(format(robot.estimation[0], '.6f')) + " " + str(format(robot.estimation[1], '.6f')) + " " + str(format(robot.estimation[2], '.6f'))))
            estimation_file.write("\n")
            odometry_file.write((str(format(robot.odometry[0], '.6f')) + " " + str(format(robot.odometry[1], '.6f')) + " " + str(format(robot.odometry[2], '.6f'))))
            odometry_file.write("\n")
            biased_trajectory_file.write((str(format(robot.biased_trajectory[0], '.6f')) + " " + str(format(robot.biased_trajectory[1], '.6f')) + " " + str(format(robot.biased_trajectory[2], '.6f'))))
            biased_trajectory_file.write("\n")
        
        event = pygame.event.poll()
        
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE) or frames >= 7500:
            for item in file_list:
                item.close()
            break
        
        # background
        screen.fill((50, 55, 60))   
        
        # Draw goal
        for goal in goalX:
            pygame.draw.circle(screen, (0,255,0), goal, 8, 0)  
        
        
        robots.robots_display(screen)
        robots.update_distance_table()      
        robots.robots_simulation_loop(goalX, K_filter, camera, SEED)
            
        
        # To limit fps, controls speed of the animation
        clock.tick(800)
        # calculate current fps     
        fps = (frames*1000)/(pygame.time.get_ticks() - ticks)   
        
        # Update PyGame display
        pygame.display.flip()
        frames += 1
