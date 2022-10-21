import numpy as np
import math

# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Extended Kalman Filter example (two-wheeled mobile robot)
 
# Supress scientific notation when printing NumPy arrays
# np.set_printoptions(precision=3,suppress=True)

class Kalman:
    def __init__(self):
        # A_k_1
        self.A_k_1 = np.array([[1.0,   0,   0],
                               [  0, 1.0,   0],
                               [  0,   0, 1.0]])
        # state_noise
        self.process_noise_v_k_minus_1 = np.array([0.01, 0.01, 0.003])

        self.Q_k = np.array([[1.0,   0,   0],
                             [  0, 1.0,   0],
                             [  0,   0, 1.0]])
             
        self.H_k = np.array([[1.0,   0,   0],
                             [  0, 1.0,   0],
                             [  0,   0, 1.0]])
                        
        self.R_k = np.array([[1.0,   0,    0],
                             [  0, 1.0,    0],
                             [  0,    0, 1.0]])  
        self.sensor_noise_w_k = np.array([0.07, 0.07, 0.04])
 
    def getB(self, phi, dk):
        B_K = np.array([[math.cos(phi)*dk, 0],
                        [math.sin(phi)*dk, 0],
                        [0, dk]])
        return B_K
 
    def EKF(self, z_k, state_estimate_k_1, u_k_1, P_k_1, dk):

        ######################### Predict #############################
        # Predict the state estimate at time k based on the state 
        # estimate at time k-1 and the control input applied at time k-1.
        state_estimate_k = self.A_k_1 @ (state_estimate_k_1) + (self.getB(state_estimate_k_1[2],dk)) @ (u_k_1) + (self.process_noise_v_k_minus_1)
                
        print(f'State Estimate Before EKF={state_estimate_k}')
                
        # Predict the state covariance estimate based on the previous
        # covariance and some noise
        P_k = self.A_k_1 @ P_k_1 @ self.A_k_1.T + (self.Q_k)
            
        ################### Update (Correct) ##########################
        # Calculate the difference between the actual sensor measurements
        # at time k minus what the measurement model predicted 
        # the sensor measurements would be for the current timestep k.
        measurement_residual_y_k = z_k - ((self.H_k @ state_estimate_k) + (self.sensor_noise_w_k))
    
        print(f'Observation={z_k}')
                
        # Calculate the measurement residual covariance
        S_k = self.H_k @ P_k @ self.H_k.T + self.R_k
            
        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be
        # non-square or singular.
        K_k = P_k @ self.H_k.T @ np.linalg.pinv(S_k)
            
        # Calculate an updated state estimate for time k
        state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
        
        # Update the state covariance estimate for time k
        P_k = P_k - (K_k @ self.H_k @ P_k)
        
        # Print the best (near-optimal) estimate of the current state of the robot
        print(f'State Estimate After EKF={state_estimate_k}')
    
        # Return the updated state and covariance estimates
        return state_estimate_k, P_k
     
def main():
 
    # We start at time k=1
    k = 1
     
    # Time interval in seconds
    dk = 1
 
    # Create a list of sensor observations at successive timesteps
    # Each list within z_k is an observation vector.
    z_k = np.array([[4.721, 0.143, 0.006], # k=1
                    [9.353, 0.284, 0.007], # k=2
                    [14.773, 0.422, 0.009],# k=3
                    [18.246, 0.555, 0.011], # k=4
                    [22.609, 0.715, 0.012]])# k=5
                     
    # The estimated state vector at time k-1 in the global reference frame.
    # [x_k_minus_1, y_k_minus_1, yaw_k_minus_1]
    # [meters, meters, radians]
    state_estimate_k_1 = np.array([0.0, 0.0, 0.0])
     
    # The control input vector at time k-1 in the global reference frame.
    # [v, yaw_rate]
    # [meters/second, radians/second]
    # In the literature, this is commonly u.
    # Because there is no angular velocity and the robot begins at the 
    # origin with a 0 radians yaw angle, this robot is traveling along 
    # the positive x-axis in the global reference frame.
    u_k_1 = np.array([4.5, 0.0])
     
    # State covariance matrix P_k_1
    # This matrix has the same number of rows (and columns) as the 
    # number of states (i.e. 3x3 matrix). P is sometimes referred
    # to as Sigma in the literature. It represents an estimate of 
    # the accuracy of the state estimate at time k made using the
    # state transition matrix. We start off with guessed values.
    P_k_1 = np.array([[0.1,   0,   0],
                      [  0, 0.1,   0],
                      [  0,   0, 0.1]])
                             
    # Start at k=1 and go through each of the 5 sensor observations, 
    # one at a time. 
    # We stop right after timestep k=5 (i.e. the last sensor observation)
    for k, obs_vector_z_k in enumerate(z_k, start=1):
     
        # Print the current timestep
        print(f'Timestep k={k}')  
         
        # Run the Extended Kalman Filter and store the 
        # near-optimal state and covariance estimates
        optimal_state_estimate_k, covariance_estimate_k = ekf(
            obs_vector_z_k, # Most recent sensor measurement
            state_estimate_k_1, # Our most recent estimate of the state
            u_k_1, # Our most recent control input
            P_k_1, # Our most recent state covariance matrix
            dk) # Time interval
         
        # Get ready for the next timestep by updating the variable values
        state_estimate_k_1 = optimal_state_estimate_k
        P_k_1 = covariance_estimate_k
         
        # Print a blank line
        print()
 
# Program starts running here with the main method  
main()