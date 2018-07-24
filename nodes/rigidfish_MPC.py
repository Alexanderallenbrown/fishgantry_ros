from numpy import *
from matplotlib.pyplot import *
from rigidfish import RigidFish
from scipy.optimize import minimize

class Fish_Planar_MPC:
    def __init__(self,Np = 100,dtp = 0.01,q_x_error = 1.0,q_psi_error = 1.0,q_vel_final_error = 1.0):
        #number of prediction timesteps used in each optimization
        self.Np = Np
        #prediction timestep
        self.dtp = dtp
        #weight on final location error
        self.Q_x_error = zeros(self.Np)
        self.Q_x_error[-1] = q_x_error
        #weight on final yaw angle
        self.Q_psi_error = zeros(self.Np)
        self.Q_psi_error[-1] = q_psi_error
        #weight on final velocity
        self.Q_vel_final_error = zeros(self.Np)
        self.Q_vel_final_error[-1] = q_vel_final_error

        self.predicted_x = zeros(self.Np)
        self.predicted_y = zeros(self.Np)
        self.predicted_psi = zeros(self.Np)

    def predict(self,fishnow, amplitude_input,bias_input,frequency_input):
        ''' this method takes the current "real" fish state, and predicts its trajectory for an input history given by three vectors of lenght Np.
        '''
        predicted_x = zeros(self.Np)
        predicted_y = zeros(self.Np)
        predicted_psi = zeros(self.Np)

        xsim = zeros((self.Np,6))
        