from numpy import *
from matplotlib.pyplot import *
from rigidfish import RigidFish
from scipy.optimize import minimize
import copy

class Fish_Planar_MPC:
    def __init__(self,Np = 100,dtp = 0.01,q_x_error = 1.0,q_y_error=1.0,q_psi_error = 1.0,q_vel_final_error = 1.0,freqmax=24.0,ampmax = 45*pi/180,biasmax = 45*pi/180):
        #number of prediction timesteps used in each optimization
        self.Np = Np
        #prediction timestep
        self.dtp = dtp
        #weight on final location error
        self.Q_x_error = ones(self.Np)
        self.Q_x_error[-1] = q_x_error
        #weight on final location error
        self.Q_y_error = ones(self.Np)
        self.Q_y_error[-1] = q_y_error
        #weight on final yaw angle
        self.Q_psi_error = ones(self.Np)
        self.Q_psi_error[-1] = q_psi_error
        #weight on final velocity
        self.Q_vel_final_error = zeros(self.Np)
        self.Q_vel_final_error[-1] = q_vel_final_error

        self.predicted_x = zeros(self.Np)
        self.predicted_y = zeros(self.Np)
        self.predicted_psi = zeros(self.Np)

        self.freqmax = freqmax
        self.ampmax = ampmax
        self.biasmax = biasmax

    def predict(self,fishnow, frequency_input, amplitude_input,bias_input):
        ''' this method takes the current "real" fish state, and predicts its trajectory for an input history given by three vectors of lenght Np.
        '''
        predicted_x = zeros(self.Np)
        predicted_y = zeros(self.Np)
        predicted_psi = zeros(self.Np)

        predictfish = copy.deepcopy(fishnow)

        xpred = zeros((self.Np,6))
        for k in range(0,len(xpred)):
            xpred[k,:] = predictfish.driveFish(self,frequency_input[k],amplitude_input[k],bias_input[k],self.dtp)
        return xpred

    def ObjectiveFn_targetPose(self,input_matrix,fishnow,target_pose):
        ''' This method takes a possible input vector [freq,amp,bias], a target pose [x_des,y_des,psi_des,vel_final]
            and then it returns the objective function value. it uses the predict() method to calculate fish states.

        '''
        #pull out individual channels
        input_reshaped = input_matrix.reshape(3,self.Np)
        #print input_reshaped.shape
        frequency_input = input_reshaped[0,:]
        amplitude_input = input_reshaped[1,:]
        bias_input = input_reshaped[2,:]
        #create a Npx6 matrix representing simulated fish states
        xpred = self.predict(fishnow,frequency_input,amplitude_input,bias_input)
        J = 0 #initialize the objective to zero
        #now loop through and update J for every timestep in the prediction horizon.
        for k in range(0,self.Np): #compute for each step in the prediction horizon
            J = J + self.Q_x_error[k]*(xpred[k,3]-target_pose[0])**2+self.Q_y_error[k]*(xpred[k,4]-target_pose[1])**2+self.Q_psi_error[k]*(xpred[k,5]-target_pose[2])**2#+self.Q_vel_final_error[k]*((sqrt(xpred[k,0]**2+xpred[k,1]**2)-target_pose[3])**2)
        return float(J)

    def calcOptimal(self,fishnow,target_pose):
        initfreq = self.freqmax*ones(self.Np)
        initamp = self.ampmax*ones(self.Np)
        initbias = zeros(self.Np)
        input_matrix = hstack((hstack((initfreq,initamp)),initbias))
        #set up constraint on optimization
        #constraint_obj = ({'type':'ineq','fun':constrain_y})
        bounds = [(0,self.freqmax)]
        for ind in range(1,len(input_matrix)):
            if ind<self.Np:
                bounds.insert(0,(0,self.freqmax))
            elif ind<self.Np*2:
                bounds.insert(1,(0,self.ampmax))
            else:
                bounds.insert(2,(-self.biasmax,self.biasmax))
        #now we use the minimize function.
        umpc = minimize(self.ObjectiveFn_targetPose,input_matrix,args=(fishnow,target_pose),method='SLSQP')
        #print umpc
        inputs = array([umpc.x])
        inputs_reshaped = inputs.reshape((3,self.Np))
        #print inputs_reshaped[0,:]
        return inputs_reshaped[:,0]#,inputs_reshaped


class Fish_EndPoint_MPC:
    ''' this is like the above class, but it ONLY considers the error at the FINAL TIME'''
    def __init__(self,Np = 100,dtp = 0.01,q_x_error = 1.0,q_y_error=1.0,q_psi_error = 1.0,q_vel_final_error = 1.0,freqmax=24.0,ampmax = 45*pi/180,biasmax = 45*pi/180):
        #number of prediction timesteps used in each optimization
        self.Np = Np
        #prediction timestep
        self.dtp = dtp
        #weight on final location error
        self.Q_x_error = q_x_error
        #weight on final location error
        self.Q_y_error = q_y_error
        #weight on final yaw angle
        self.Q_psi_error = q_psi_error
        #weight on final velocity
        self.Q_vel_final_error = q_vel_final_error

        self.freqmax = freqmax
        self.ampmax = ampmax
        self.biasmax = biasmax

    def predict(self,fishnow, frequency_input, amplitude_input,bias_input):
        ''' this method takes the current "real" fish state, and predicts its trajectory for an input history given by three vectors of lenght Np.
        '''
        predicted_x = zeros(self.Np)
        predicted_y = zeros(self.Np)
        predicted_psi = zeros(self.Np)

        predictfish = copy.deepcopy(fishnow)

        xpred = zeros((self.Np,6))
        for k in range(0,len(xpred)):
            xpred[k,:] = predictfish.driveFish(self,frequency_input,amplitude_input,bias_input,self.dtp)
        return xpred

    def ObjectiveFn_targetPose(self,input_matrix,fishnow,target_pose):
        ''' This method takes a possible input vector [freq,amp,bias], a target pose [x_des,y_des,psi_des,vel_final]
            and then it returns the objective function value. it uses the predict() method to calculate fish states.

        '''

        #print input_reshaped.shape
        frequency_input = input_matrix[0]
        amplitude_input = input_matrix[1]
        bias_input = input_matrix[2]
        #create a Npx6 matrix representing simulated fish states
        xpred = self.predict(fishnow,frequency_input,amplitude_input,bias_input)
        #now compute objective function
        J = self.Q_x_error*(xpred[-1,3]-target_pose[0])**2+self.Q_y_error*(xpred[-1,4]-target_pose[1])**2+self.Q_psi_error*(xpred[-1,5]-target_pose[2])**2#+self.Q_vel_final_error[k]*((sqrt(xpred[k,0]**2+xpred[k,1]**2)-target_pose[3])**2)
        return float(J)

    def calcOptimal(self,fishnow,target_pose):
        initfreq = self.freqmax
        initamp = self.ampmax
        initbias = random.randn(1)
        input_matrix = array([initfreq,initamp,initbias])
        #set up constraint on optimization
        #constraint_obj = ({'type':'ineq','fun':constrain_y})
        bounds = [(0,self.freqmax)]
        bounds.insert(0,(0,self.ampmax))
        bounds.insert(0,(-self.biasmax,self.biasmax))
        
        #now we use the minimize function.
        umpc = minimize(self.ObjectiveFn_targetPose,input_matrix,args=(fishnow,target_pose),bounds=bounds,method='SLSQP')
        #print umpc
        inputs = array([umpc.x])
        #inputs_reshaped = inputs.reshape((3,self.Np))
        #print inputs_reshaped[0,:]
        print inputs[0]
        return list(inputs[0])#,inputs_reshaped

def demo():
    fish = RigidFish()
    freq = 20.0*ones(100)
    amp = 10.0*pi/180.0*ones(100)
    bias = 0.0*ones(100)
    input_matrix = zeros((100,3))
    input_matrix[:,0] = freq
    input_matrix[:,1] = amp
    input_matrix[:,2] = bias

    fishbrain = Fish_EndPoint_MPC()

    target_pose = [0.5,0.5,0.0,0.0]
    # J =fishbrain.ObjectiveFn_targetPose(input_matrix,fish,target_pose)
    # print J
    # inputs_now,total_inputs = fishbrain.calcOptimal(fish,target_pose)
    simtime = 2 #seconds
    dT = .01
    t = arange(0,simtime,dT) #time vector
    fishstates_store = zeros((len(t),6))
    inputs_store = zeros((len(t),3))

    for k in range(0,len(t)):
        #calculate the inputs right meow.
        freq,amp,bias = fishbrain.calcOptimal(fish,target_pose)
        inputs_store[k,:] = array([freq,amp,bias])
        #print freq,amp,bias
        #update the fish using these inputs
        fishstates_store[k,:] = fish.driveFish(freq,amp,bias,dT)
        #print t[k]


    figure()
    plot(t,inputs_store[:,1])

    figure()
    plot(fishstates_store[:,3],fishstates_store[:,4],'k')
    axis('equal')

    show()

    

if __name__ == '__main__':
    demo()
        