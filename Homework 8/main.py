import numpy as np
from forward_kinematics import forward_kinematics_product_exp
from control_utils import twist_matrix, adjoint_matrix

def main():
    # Example joint angles for a 3-DOF robot arm
    joint_angles = [np.pi/3, -np.pi/4, -np.pi/2]  # Replace with actual joint angles

    # Compute forward kinematics
    g_wf, twist = forward_kinematics_product_exp(joint_angles)

    print("End Effector Configuration (g_wf):")
    print(g_wf)
    print("Vector form of twist:")
    print(twist)

    twist1 = np.array([0, 0, 0, 0, 0, 1])
    #twist_hat2 = twist_matrix(, twist[3:6])
    #twist_hat3 = twist_matrix(twist[3:6], twist[0:3])
    

if __name__ == "__main__":
    main()