import numpy as np  # for simple operations
from scipy.integrate import solve_ivp   # for integrate the equation of motion 
import matplotlib.pyplot as plt   # for plot figure and animation 

# Write Given information like mass, time , initial and final state and other information
m2 = 1.5
l2 = 1.5
g = 9.81

q_0 = #np.array([,   ])      # [theta1, d2] at t = 0 (Lab-8)
q_T = #np.array([ , ])       # [theta1, d2] at t = T (Lab-8)
# write a function for transformation matrix from dh parameters
# DH forward kinematics
# DH row convention: [alpha(i-1), a(i-1), d(i), theta(i)]
def dh2T(al, a, d, th):
    return #a NumPy array with shape(4, 4). the 4x4 homogeneous transformation matrix for one DH row.
def fk(th1, d2):
    #Calculate transformations T01 and T02 for the R-P manipulator.
    T01 = dh2T() #for joint 1 (revolute).
    T12 = dh2T() #for joint 2 (prismatic).
    T02 = T01 @ T12
    #Multiply the matrices in the correct order to obtain T02.
    return T01, T02
    

def traj(q0, qT, T, t):
    #Return q, qd, qdd at time t for a cycloidal trajectory over duration T.
    #Inputs:
    #q0: initial joint coordinate, qT: final joint coordinate, t: time
    #Implement the cycloidal position, velocity, and acceleration formulas.
    q = ...
    qd = ...
    qdd = ...
    return q, qd, qdd

def lab8_torque(t):
    #Calculate the torque at joint 1 and force at joint 2 for the prescribed cycloidal trajectory.
    #Call traj() for theta1 and for d2.
    #Unpack position, velocity, and acceleration for both joints.
    #Implement the Lab-8 inverse-dynamics equations:
    tau1 = ...
    f2   = ...
    return np.array([tau1, f2])

def derivativevstatefreefall(t, x):#x is received as argument
    th1, d2, th1d, d2d = x
    I = np.array([[m2*d2**2, 0],
                  [0,        m2]])
    C = np.array([[ m2*d2*d2d,   m2*d2*th1d],
                  [-m2*d2*th1d,  0]])
    h = np.array([m2*d2*np.sin(th1)*g,
                  -m2*np.cos(th1)*g])
    tau = np.zeros((2)) # the torque for free fall
    qdd = np.linalg.solve(I, tau - C @ [th1d, d2d] - h)
    return [th1d, d2d, qdd[0], qdd[1]] #xdot is returned

def derivativevstate(t, x):#x is received as argument
    th1, d2, th1d, d2d = x
    I = np.array([[m2*d2**2, 0],
                  [0,        m2]])
    C = np.array([[ m2*d2*d2d,   m2*d2*th1d],
                  [-m2*d2*th1d,  0]])
    h = np.array([m2*d2*np.sin(th1)*g,
                  -m2*np.cos(th1)*g])
    tau = lab8_torque(t) # the torque is calculated
    qdd = np.linalg.solve(I, tau - C @ [th1d, d2d] - h)
    return [th1d, d2d, qdd[0], qdd[1]] #xdot is returned
    

x0= np.array([np.deg2rad(0),0.5,0,0]) #Initial values
tar=np.linspace(0,T,50) #time steps
sol = solve_ivp(derivativevstate, (0, T), x0,t_eval=tar) #Numerical intergation. Use derivativevstatefreefall for free fall
th1, d2, th1d, d2d = sol.y #solution for x
t = sol.t #time instance

th1dd, d2dd = [], []
for k in range(len(t)):
    xdot = derivativevstate([k], [th1[k], d2[k], th1d[k], d2d[k]])
    th1dd.append(xdot[2])
    d2dd.append(xdot[3])

#Plots
#theta and d2 vs time, t
#Animations
#Variation of EE in the workspace
