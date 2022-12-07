import numpy as np
import math
import random

SEED = 100
N = 10
screen_width = 1080
screen_height = 640

if(__name__ == '__main__'):
    np.random.seed(SEED)
    
    # phi = np.random.randint(1000000, size=(1, N*2000))
    # print(phi)
    goalX = np.random.randint(640, size=(1, N*2000))
    print(goalX)
    
    # i = 0
    # while(i < 10):
    #     phi = np.round(100*np.random.rand(N)) / 10
    #     print(phi)
    #     i = i + 1
    
    