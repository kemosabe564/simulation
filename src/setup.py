import numpy as np
import math

# init
PI = math.pi
MODE = "sr_EKF"


# "normal" "sr_EKF" "mr_EKF"

# screen
screen_width = 1080
screen_height = 640

# robot
N = 10
robot_l = 15; robot_b = 6 
robots_x = np.array([100, 100, 100, 200, 200, 300, 400, 500, 700, 900])
robots_y = np.array([100, 200, 400,  50, 300, 500, 300, 400, 100, 400])

# robots_x = np.array([100, 100, 150])#, 200, 200, 300, 400, 500, 700, 900])
# robots_y = np.array([100, 150, 100])#,  50, 300, 500, 300, 400, 100, 400])    

robots_phi = 0.00 * np.array([PI, PI, PI, PI, PI, PI, PI, PI, PI, PI])

omega0 = np.array([0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0])

robots_num = len(robots_x)
goalX = np.array([[600, 400], [600, 100], [900, 300], [400, 500], [600, 400], [1000, 500], [1000, 100], [600, 600], [100, 100], [100, 400]])    # goal position

setup = {"vmax":0.5, "gtg_scaling":0.0001, "K_p":0.01, "ao_scaling":0.00005}
