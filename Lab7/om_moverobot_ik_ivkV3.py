import rospy
import numpy as np
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

############################################
#Inverse Kinematics code
###########################################
def Rzyx(orient):#ZYX Euler angle
    cz, sz = np.cos(orient[2]), np.sin(orient[2])
    cy, sy = np.cos(orient[1]), np.sin(orient[1])
    cx, sx = np.cos(orient[0]), np.sin(orient[0])
    Rzyx=np.array([
        [cz, -sz, 0],
        [sz,  cz, 0],
        [0,  0, 1]
    ])@np.array([
        [ cy, 0, sy],
        [ 0, 1, 0],
        [-sy, 0, cy]
    ])@np.array([
        [1, 0,  0],
        [0, cx, -sx],
        [0, sx,  cx]
    ])
    return Rzyx 
    
def within_limits(q1, q2, q3, q4, margin=0.05):
    """margin (rad) absorbs real-robot calibration slop around the nominal
    URDF limits -- tune down to 0.0 if you want to enforce them strictly."""
    # Joint limits, straight from the URDF <limit> tags (radians)
    JOINT_LIMITS = {
    1: (-0.9 * np.pi, 0.9 * np.pi),
    2: (-0.57 * np.pi, 0.5 * np.pi),
    3: (-0.3 * np.pi, 0.44 * np.pi),
    4: (-0.57 * np.pi, 0.65 * np.pi),
    }
    for i, q in enumerate([q1, q2, q3, q4], start=1):
        lo, hi = JOINT_LIMITS[i]
        if not (lo - margin <= q <= hi + margin):
            return False
    return True    


def geometric_ik_all(P, R, filter_by_limits=True, limit_margin=0.05):
    """
    P : desired end-effector position (Px, Py, Pz), world/frame0
    R : desired end-effector orientation as a 3x3 rotation matrix (= R0E = R04,
        since the tool offset carries no extra rotation)
    filter_by_limits : if True, only return branches within JOINT_LIMITS
                        (+/- limit_margin). This arm has no wrist roll, so
                        joint limits alone are usually enough to pick out
                        the single physically valid branch.
    Returns a list of (q1, q2, q3, q4) solutions in radians, each already
    wrapped so q1 in (-pi, pi].
    """
    A0 = 0.012                       # fixed x-offset of the shoulder pivot from the yaw axis
    D1 = 0.017 + 0.0595              # fixed height of the shoulder pivot (0.0765)
    A2 = np.hypot(0.024, 0.128)      # straight-line joint2->joint3 length (0.13023)
    A3 = 0.124                       # joint3->joint4 length (no offset angle needed here)
    A4 = 0.126                       # tool offset, joint4->end-effector
    
    # Mechanical offset angle from the joint2/joint3 bracket geometry (0.024, 0.128)
    BETA = np.arctan2(0.024, 0.128)
    OFFSET = np.pi / 2 - BETA


    P = np.asarray(P, dtype=float)
    X4 = R[:, 0]                      # tool x-axis direction in world coords
    O4 = P - A4 * X4                  # wrist center

    d = O4 - np.array([A0, 0.0, D1])  # relative to fixed shoulder pivot
    r = np.hypot(d[0], d[1])
    h = d[2]
    q1_fwd = np.arctan2(d[1], d[0])
    q1_back = q1_fwd + np.pi

    solutions = []
    for q1_i, r_i in [(q1_fwd, r), (q1_back, -r)]:
        D = np.clip((r_i**2 + h**2 - A2**2 - A3**2) / (2*A2*A3), -1.0, 1.0)
        for s in (+1, -1):
            bend = np.arctan2(s * np.sqrt(1 - D**2), D)   # bend = -theta3'
            psi1 = (np.arctan2(h, r_i)
                    - np.arctan2(A3 * np.sin(bend), A2 + A3 * np.cos(bend)))

            Xr = X4[0] * np.cos(q1_i) + X4[1] * np.sin(q1_i)
            Xz = X4[2]
            psi4 = np.arctan2(Xz, Xr)     # <-- fixed: was pi/2 - arccos(dot(X4,Z0))

            q2 = -psi1 + OFFSET
            q3 = -bend - OFFSET
            q4 = -psi4 + (psi1 + bend)

            q1_wrapped = (q1_i + np.pi) % (2 * np.pi) - np.pi
            solutions.append((q1_wrapped, q2, q3, q4))

    if filter_by_limits:
        filtered = [s for s in solutions if within_limits(*s, margin=limit_margin)]
        if filtered:
            return filtered
        # nothing passed -- fall back to returning everything so the caller
        # can still see the candidates (e.g. target may be near a limit edge)
        return solutions
    return solutions

########################################


##########################
#Inverse Velocity Kinematics code
##############################
# ---- arm definition: modified DH rows [alpha, a, d, theta_offset], all revolute ----
# add/remove rows -> any number of links. Nothing else in the code depends on n.
OFFSET = np.pi / 2 - np.arctan2(0.024, 0.128)
DH = [[0,          0.012,             0.077,            0],
      [-np.pi / 2, 0.000,                 0,      -OFFSET],
      [0,          0.130,                 0,       OFFSET],
      [0,          0.124,                 0,            0]]
TOOL = np.array([0.126, 0, 0])            # end-effector point in last frame

# which of [x, y, z, phi, theta, psi] to control (1 = control)
# keep #controlled <= n.  4-DOF arm: x,y,z,pitch.  6-DOF arm: all six.
MASK = np.array([1, 1, 1, 0, 1, 0], bool)


#DH parameter to Transformation matrix
#Ti-1 to i tranform using DH parameters alpha, a, d, theta as input
#output 4 x 4 numpy array of Ti-1 to i tranform 
def dh(alpha, a, d, th):
    #To be filled!
    return np.array([])#to be edited

#Rotation matrix to ZYX Euler angles
#Input is an numpy array given input Rotation matrix R which is a numpy array 
#Output is a numpy array of Euler angles  
def euler(R):
    """phi, theta, psi for R = Rz(psi) Ry(theta) Rx(phi)"""
    return np.array([])#to be edited

#Vector of Joint angles to intermediate frame poses and end-effector pose 
#input numpy array of joint angles, 
#output is 1) list of transformations of each frames and 2) the final end-effector pose with orientation in terms of Euler ZYX angles
def fk(q):
    """frames of every joint, and end-effector pose [x, y, z, phi, theta, psi]"""
     #To be filled!
    return frames, np.concatenate([p, euler(T[:3, :3])])


#Vector of joint angles to linear and angular velocity Jacobians
#input is a numpy array of joint angles, 
#output is Jv and Jw the linear velocity Jacobina and angular velocity Jacobian
def jacobians(q):
    """geometric Jacobians Jv, Jw (3 x n)"""
    #To be filled!
    return Jv, Jw

#Euler angles to A matrix
# input is phi, and theta the Euler angles about Z and Y axes 
#ouput is numpy array A that converts the Euler angles rates to angular velocity
def euler_A(theta, psi):
    """omega = A [phi_dot, theta_dot, psi_dot]"""
    return np.array([])#to be edited

#Vector of joint angles to task space velocity Jacobian
#input is numpy array of joint angles, 
#output is Jacobian that relates linear velocity and theta rate to vector of joint angle rates
def task_jacobian(q):
    """[Jv ; A^-1 Jw] (6 x n), rows selected by MASK"""
    #To be edited
    return np.vstack([Jv, np.linalg.solve(A, Jw)])[MASK]

#Main Inverse Kinematics solution for target pose given current pose
#input is target which is a list of position coordinates and Euler angles to be attained, list of joint angles of the surrent pose, number of maximum iterations
#Output is the joint values of the target pose
def ik(target, q, iters=300):
    """target = [x, y, z, phi, theta, psi] (only MASK entries are used)"""
    q = np.array(q, float)
    for _ in range(iters):
        _, pose = fk(q)
        e = np.asarray(target, float) - pose
        #e[3:] = (e[3:] + np.pi) % (2 * np.pi) - np.pi      # wrap angles
        e = e[MASK]
        if np.linalg.norm(e) < 1e-8:
            break
        J = task_jacobian(q)
        #q += J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(len(e)), e)
        q += np.linalg.solve(J, e)
    return q
    
#####################################################    


#############################################
#Service based movement of robot
#########################################

def move_robot(jval,pathtime):
    rospy.init_node('open_manipulator_service_client')
    rospy.loginfo('Waiting for /goal_joint_space_path service...')
    rospy.wait_for_service('/goal_joint_space_path')
    try:
        move_joint = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
        req=SetJointPositionRequest()
        req.planning_group='arm'
        req.joint_position.joint_name = ['joint1','joint2', 'joint3', 'joint4']
        req.joint_position.position = jval
        req.joint_position.max_accelerations_scaling_factor=0.2
        req.joint_position.max_velocity_scaling_factor=0.3
        req.path_time=pathtime
        response = move_joint(req)
        rospy.loginfo('Trajectory planned: %s',response.is_planned)
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s',e)
######################################        

if __name__ == '__main__':

    pe1 = [0.05, 0.00, 0.20, 0, 0, 0]#pose 1 as postioni and Euler ZYX angles
    pe2 = [0.15, 0.00, 0.20, 0, np.radians(45), 0]#pose 2 as postion and Euler ZYX angles
    
    #Inverse Kinematics to move to first pose pE1
    pe1 = np.array(pe1)
    Re1 = Rzyx(pe1[3:])
    ##joint_states =  [-,-,-,-] 
    sols = geometric_ik_all(pe1[:3], Re1)
    print('Solutions: ',sols)
    print('Moving to the first joint configuration: ',sols[0])
    move_robot(sols[0],3.0)
    rospy.sleep(3.0)	
    
    #Inverse velocity Kinematics to move to second pose pE2
    q2=ik(pe2,sols[0])
    print('Moving to solution for 2-nd pose: ',q2)
    move_robot(q2.tolist(),3.0)   
