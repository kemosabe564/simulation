from distutils.core import setup
import numpy as np
import pygame
from pygame.locals import *

from src.Robots import *
from src.Kalman import *
from src.Camera import *

if(__name__ == '__main__'):
    
    # open the file
    
    measurement_Kalman_file = open('data\\measurement_Kalman.txt', 'w')
    measurement_true_file = open('data\\measurement_true.txt', 'w')
    measurement_bias_file = open('data\\measurement_bias.txt', 'w')
    estimation_file = open('data\\estimation.txt', 'w')
    desired_trajectory_file = open('data\\desired_trajectory.txt', 'w')
    biased_trajectory_file = open('data\\biased_trajectory.txt', 'w')
    file_list = [measurement_Kalman_file, measurement_true_file, measurement_bias_file, estimation_file, desired_trajectory_file, biased_trajectory_file]
    
    # init
    screen_width = 1080; screen_height = 640
    screen = pygame.display.set_mode([screen_width, screen_height], DOUBLEBUF)
    
    robot_l = 15; robot_b = 6 
    goalX = np.array([[600, 400], [600, 100], [900, 300]])#, [400, 500], [600, 400], [1000, 500], [1000, 100], [600, 600], [100, 100], [100, 400]])    # goal position
    
    setup = {"vmax":0.5, "gtg_scaling":0.0001, "K_p":0.01, "ao_scaling":0.00005}
    
    # init robots
    robots_x = np.array([100, 100, 100])#, 200, 200, 300, 400, 500, 700, 900])
    robots_y = np.array([100, 200, 400])#,  50, 300, 500, 300, 400, 100, 400])
    robots_phi = np.array([0, 0, 0])# , 0, 0, 0, 0, 0, 0, 0])
    
    robots_num = len(robots_x)
    
    robots = Robots(robots_num, robots_x, robots_y, robots_phi, goalX, setup)
    
    K_filter = Kalman()    
    
    camera = Camera(robots_num, robots_x, robots_y, robots_phi)
    
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
            desired_trajectory_file.write((str(format(robot.desired_trajectory[0], '.6f')) + " " + str(format(robot.desired_trajectory[1], '.6f')) + " " + str(format(robot.desired_trajectory[2], '.6f'))))
            desired_trajectory_file.write("\n")
            biased_trajectory_file.write((str(format(robot.biased_trajectory[0], '.6f')) + " " + str(format(robot.biased_trajectory[1], '.6f')) + " " + str(format(robot.biased_trajectory[2], '.6f'))))
            biased_trajectory_file.write("\n")
        
        # print(robot.measurement_true)
        event = pygame.event.poll()
        
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE) or frames >= 10000:
            for item in file_list:
                item.close()
            break
        
        screen.fill((50, 55, 60))   # background
        
        for goal in goalX:
            pygame.draw.circle(screen, (0,255,0), goal, 8, 0)  # Draw goal
        
        
        robots.robots_display(screen)
        robots.update_distance_table()
        
        
        robots.robots_simulation_loop(goalX, K_filter, camera)
            
        
        
        clock.tick(500)     # To limit fps, controls speed of the animation
        fps = (frames*1000)/(pygame.time.get_ticks() - ticks)   # calculate current fps
        
        # Update PyGame display
        pygame.display.flip()
        frames += 1
