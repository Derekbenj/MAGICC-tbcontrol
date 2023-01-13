import math

import numpy as np
from scipy.integrate import odeint

from scipy import interpolate

import sys
sys.path.append('/home/wderekb/MAGICC_TB_LEVELSET/control_testing/src/keys/')
from time import sleep

from scripts.controller import controller, spline_seg, plot_spline
import matplotlib.pyplot as plt

import rospy
from scripts.ros_control_node import ros_control_node


turnRateHistory = []
velocityHistory = []
def saturate_input(value, range):
    if value < range[0]:
        return range[0]
    if value  > range[1]:
        return range[1]
    return value

def dynamics(state, time, input, turn_rate_range, velocity_range):
# def dynamics(state, input, turn_rate_range, velocity_range):
    turn_rate = saturate_input(input[0], turn_rate_range)
    velocity = saturate_input(input[1], velocity_range)
    turnRateHistory.append(turn_rate)
    velocityHistory.append(velocity)
    theta = state[2]
    
    x_dot = velocity * np.cos(theta) 
    y_dot = velocity * np.sin(theta) 

    theta_dot = turn_rate

    return np.array([x_dot, y_dot, theta_dot])


def integrate_dynamics(state, input, dt, turn_rate_range, velocity_range):
    time = [0,dt]
    return odeint(dynamics, state, time, args=(input,turn_rate_range,velocity_range))[1]


def run_simulation(controller, ros_node, time, timeStep, turn_rate_range, velocity_range):
    '''
    Configure the turtlebot by spawning the new turtle named "my_turtle" and teleporting him to a random start position near 0, 0. For the amount of time calculated in our
    spline trajectory, get the turtlebot's current position and make the geometry_twist message for our turtlebot using the controller code. Publish that information to
    the topic controlling the turtlebot and ask the turtlebot for it's new position after receiving the command. Finally, update the time variable. Commented code lets you compare
    the turtlebot's actual position to a dynamics solver's expected position following the course-correcting commands.
    '''
    # initial configuration
    steps = int(time / timeStep) 

    currentState = ros_node.get_current_state()
    print("turtlebot state: ", currentState)
    t = 0
    states = []

    while t < time:
        input  = controller.get_control(np.array(currentState), t)
        ros_node.generate_twist_msg(input)
        # nextState = integrate_dynamics(currentState, input, timeStep, turn_rate_range, velocity_range) + np.random.normal(0, [0.01,0.01,0.01], 3)
        ros_node.publish_cmd_vel()
        currentState = ros_node.get_current_state()
        states.append(currentState)
        t += controller.timestep / 1000000000
        print("\ntime is: ", t, end='\n\n')

        # print("turtlebot position: ", ros_node.get_current_state())
        # print("sim       position: ", nextState)

    return np.array(states)


if __name__ == '__main__':
    # set caps on turn rate 
    turn_rate_range = [-2,2]
    velocity_range = [0.5,10]

    # start the turtle off with a random location [x, y, theta] each with a random value between 0 and 1
    # state_0 = np.random.uniform([0,0,0], [1,1,1],3)

    # control points are [x, y, theta]. x and y have max of 11
    controlPoints = [[0,0],[0.5,0.2],[0.9,0.9], [-1.2, 1.2]]
    splineTime = [0,100]

    spline = spline_seg(controlPoints, splineTime[0], splineTime[1])

    cont = controller(spline)
    ros_node = ros_control_node()

    try:
        states = run_simulation(cont, ros_node, splineTime[1], 0.01, turn_rate_range, velocity_range)
    except rospy.ROSInterruptException:
        pass

    # code for displaying error, velocities, etc. after the simulation finishes
    plt.figure()
    error = np.array(cont.error)
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(error[:,0])),error[:,0], label = 'X error')
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(error[:,0])),error[:,1], label = 'Y error')
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(error[:,0])),error[:,2], label = 'Theta error')
    plt.legend()
    
    plt.figure()
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(turnRateHistory)), turnRateHistory)
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(cont.turnRateHistory)), np.ones(len(cont.turnRateHistory))*turn_rate_range[0], color = 'r')
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(cont.turnRateHistory)), np.ones(len(cont.turnRateHistory))*turn_rate_range[1], color = 'r')
    plt.title("turn rate")

    plt.figure()
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(velocityHistory)), velocityHistory)
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(cont.turnRateHistory)), np.ones(len(cont.turnRateHistory))*velocity_range[0], color = 'r')
    plt.plot(np.linspace(splineTime[0], splineTime[1], len(cont.turnRateHistory)), np.ones(len(cont.turnRateHistory))*velocity_range[1], color = 'r')
    plt.title("velocity")

    plt.figure()
    plot_spline(spline, 31)
    plt.plot(states[:,0],states[:,1])
    plt.xlim(0, 11)
    plt.ylim(0, 11)
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    plt.show()