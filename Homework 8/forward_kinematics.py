import numpy as np
from scipy.linalg import expm
from control_utils import twist_matrix

def forward_kinematics_product_Lie(d):
    """
    Forward Kinematics
    :param d: Mujoco Data
    :return: End Effector Position
    """
    # Number of Joints
    nj = d.qpos.shape[0]
    
    # Variables for Theta 1, Theta 2, Theta 3, Theta 4, Theta 5, Theta 6
    theta1 = d.qpos[0]
    theta2 = d.qpos[1]
    theta3 = d.qpos[2]
    theta4 = d.qpos[3]
    theta5 = d.qpos[4]
    theta6 = d.qpos[5]
    
    # Obtain all Rotation Matrices
    R1 = np.eye(3) # This rotation is incorrect, but it is a placeholder
    R2 = np.eye(3)
    R3 = np.eye(3)
    R4 = np.eye(3)
    R5 = np.eye(3)
    R6 = np.eye(3)
    
    # Obtain all Displacement Vectors
    d1 = np.array([[0], [0], [0]]) # This displacement is incorrect, but it is a placeholder
    d2 = np.array([[0], [0], [0]])
    d3 = np.array([[0], [0], [0]])
    d4 = np.array([[0], [0], [0]])
    d5 = np.array([[0], [0], [0]])
    d6 = np.array([[0], [0], [0]])
    # dt = np.array([[0], [0.1], [0]])
    
    # Separately compute each frame transformation
    g_wa = np.block([[R1, d1],[0, 0, 0, 1]])
    g_ab = np.block([[R2, d2],[0, 0, 0, 1]])
    g_bc = np.block([[R3, d3],[0, 0, 0, 1]])
    g_cd = np.block([[R4, d4],[0, 0, 0, 1]])
    g_de = np.block([[R5, d5],[0, 0, 0, 1]])
    g_ef = np.block([[R6, d6],[0, 0, 0, 1]])
    # g_ft = np.block([[np.eye(3), dt],[0, 0, 0, 1]])
    
    # Compute the End Effector Configuration
    g_wf = g_wa @ g_ab @ g_bc @ g_cd @ g_de @ g_ef 
         
    return g_wf  
       
def forward_kinematics_product_exp(joint_angles):
    """
    Forward Kinematics using product of exponentials
    :param d: Mujoco Data
    :return: End Effector Position
    """
    # Number of Joints
    nj = len(joint_angles)
    
    # Variables for Theta 1, Theta 2, Theta 3, Theta 4, Theta 5, Theta 6
    thetas = []
    for i in range(nj):
        thetas.append([joint_angles[i]])
    
    # Define the reference configuration
    R0 = np.eye(3)
    d0 = np.array([[0], [1], [1]])
    g_0 = np.block([[R0, d0],[0, 0, 0, 1]])
    
    # Define all rotation axes
    omegas = []
    omegas.append(np.array([[0], [0], [1]])) # First rotation axis
    omegas.append(np.array([[-1], [0], [0]])) # Second rotation axis
    omegas.append(np.array([[0], [1], [0]])) # Third rotation axis
    
    # Define all points on the axis
    q_all = []     
    q_all.append(np.array([[0], [0], [1]])) # Point on first rotation axis
    q_all.append(np.array([[0], [0], [1]])) # Point on second rotation axis
    q_all.append(np.array([[0], [0], [1]])) # Point on third rotation axis

    
    # Compute the twists and matrix exponentials
    g_wf = np.eye(4)
    for i in range(nj):
        v = np.cross(-omegas[i].flatten(), q_all[i].flatten())
        twist = np.vstack((v.reshape((3,1)), omegas[i]))
        twist_hat = twist_matrix(v,omegas[i].flatten()) 
        twist_theta = twist_hat * thetas[i][0]
        
        g_current = expm(twist_theta)
        
        # You can instead use the following line if you'd like to see how to do the math using our formulas
        # g_current = twist_exponential_hand(omegas[i], v, thetas[i]) 
        
        g_wf = g_wf @ g_current
        
    g_wf = g_wf @ g_0
    return g_wf, twist