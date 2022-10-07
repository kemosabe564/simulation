import numpy as np
import pygame
import math
import random

class Obstacles:

    def __init__(self, screen_width, screen_height, num_circ_obsts = 2, obst_min_radius = 10, obst_max_radius = 50):
        self.num_circ_obsts = num_circ_obsts
        self.obst_min_radius = obst_min_radius
        self.obst_max_radius = obst_max_radius
        self.radius = []
        self.circ_x = []
        self.circ_y = []
        self.obstacles_list = []
        for i in range(self.num_circ_obsts):
            self.radius.append(random.randint(self.obst_min_radius, self.obst_max_radius))
            self.circ_x.append(random.randint(self.radius[i], screen_width -  self.radius[i]))
            self.circ_y.append(random.randint(self.radius[i], screen_height - self.radius[i]))
            # self.obstacles_list.append[self.radius[i], self.circ_x[i], self.circ_y[i]]
        
    def draw_circular_obsts(self, screen):
        for i in range(self.num_circ_obsts):
            pygame.draw.circle(screen, (0, 0, 255), (self.circ_x[i], self.circ_y[i]), self.radius[i], 0)
