import time
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt



class controller():
    '''
    Controller to unicycle model vehicle, takes as input a reference trajectory in the form of a b-spline, the time reference for the trajectory and 
    outputs the turnrate and velocity needed to track the b-spline trajectory using: 
    https://reader.elsevier.com/reader/sd/pii/S2405896315013890?token=9B03AF1B0F56C7100C25C958EAFD5552C8BFB7761C7B4B011F9EEA0F9EDE40359DA75A48F757D256F655E4CA13E7009A&originRegion=us-east-1&originCreation=20230102192314
    The gains can be tuned to enforce min/max velocity and turn rate constraints
    '''
    def __init__(self, spline):
        self.spline = spline

        self.gains = [10, 10, 10]

        self.error = []
        self.turnRateHistory = []
        self.velocityHistory = []

        self.time = time.time_ns()
        self.timestep = 0
    

    def set_spline(self, spline):
        self.spline = spline

    def compute_velocity_and_turn_rate_references(self, t):
        #find the first derivative of the spline for the differentailly flat model
        out_d1 = self.spline.derivative(1)(t)
        #find the second derivative of the spline
        out_d2 = self.spline.derivative(2)(t)

        #pick out the x and y components of the first and second derivative of the spine
        x1_dot = out_d1[0]
        x2_dot = out_d1[1]
        x1_ddot = out_d2[0]
        x2_ddot = out_d2[1]

        #calculate the turn rate to follow the spline
        u = (np.multiply(x1_dot, x2_ddot) - np.multiply(x2_dot, x1_ddot)) / (np.square(x1_dot) + np.square(x2_dot))
        v = np.sqrt(np.square(x1_dot) + np.square(x2_dot))

        return [u,v]
    
    def compute_state_error(self, state, referenceState, t):
        theta = state[2]
        transformMatrix = np.array([[np.cos(theta), np.sin(theta), 0],[-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        return transformMatrix @ (referenceState - state)

    def compute_reference_state(self, t):
        [xRef, yRef] = self.spline(t)

        out_d1 = self.spline.derivative(1)(t)

        #pick out the x and y components of the first and second derivative of the spine
        x1_dot = out_d1[0]
        x2_dot = out_d1[1]

        thetaRef = np.arctan2(x2_dot, x1_dot)
        return [xRef, yRef, thetaRef]


    def get_control(self, state, t):
        # updated to keep track of timesteps. The timestep is associated with the lapse in time between calls to this function in order to track
        # the time the turtlebot has been allowed to move
        self.timestep = time.time_ns() - self.time
        self.time = time.time_ns()

        turnRateReference, velocityReference = self.compute_velocity_and_turn_rate_references(t)
        stateReference = self.compute_reference_state(t)
        print(turnRateReference)
        stateError = self.compute_state_error(state, stateReference, t)
        xError = stateError[0]
        yError = stateError[1]
        thetaError = stateError[2]

        self.error.append(stateError)

        velocityControl = velocityReference + (self.gains[0] * xError) / (np.sqrt(1 + xError**2 + yError**2))
        turnRateControl = turnRateReference + (self.gains[1] * velocityReference * (yError * np.cos(thetaError / 2) - xError * np.sin(thetaError / 2))) / (np.sqrt(1 + xError**2 + yError**2)) + self.gains[2] * np.sin(thetaError / 2)

        self.turnRateHistory.append(turnRateControl)
        self.velocityHistory.append(velocityControl)
        return [turnRateControl, velocityControl]

def spline_seg(control_points,t0,tf):
    '''
    Wrapper function for scipy bspline class, this creates a clamped bpline with evenly spaced knot points (expect for first few and last few which are repeated)
        with control points and start and stop time specified by parameters
    params:
        control_points: control points of the spline
        t0: intial time of the spline (usually 0)
        tf: final time of the spline (this is changed by the optimizer
    returns:
        scipy bspline class
    '''

    #the number of control points
    l = len(control_points)

    #create evenly spaced knot points
    t = np.linspace(t0, tf, l - 2, endpoint=True)

    #add repeated knot points at begining and end
    t = np.append([t0, t0, t0], t)
    t = np.append(t, [tf, tf, tf])

    #create scipy bpline object
    spline = interpolate.BSpline(t, control_points, 3)

    return spline

def plot_spline(spl,num_points):
    t0 = spl.t[0]
    tf = spl.t[-1]
    t = np.linspace(t0, tf, num_points, endpoint=True)
    x = spl(t)[:,0]
    y = spl(t)[:,1]
    plt.plot(x,y)
    # plt.scatter(x,y,)
    control_points = spl.c
    plt.plot(control_points[:, 0], control_points[:, 1], 'k--', label='Control polygon', marker='o')