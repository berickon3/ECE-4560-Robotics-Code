import numpy as np
import matplotlib.pyplot as plt

def get_axes(p, R):
    x_axis = p + R @ np.array([[1], [0]])
    y_axis = p + R @ np.array([[0], [1]])
    return x_axis, y_axis

def R(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]])

def visualize_robot(joint_angles, link_lengths):
    
    # Get all frame rotation matrices
    Rlist = []
    
    # Get all frame displacement vectors
    dlist = []
    
    # Iteratively compute the homogeneous coordinates of each frame and extract the position and rotation matrices for each frame
    frame_pos = []
    frame_rot = []

    # Plot the robot arm as a series of frame locations
    plt.figure()
    plt.plot([coord[0] for coord in frame_pos], [coord[1] for coord in frame_pos], 'ko-')
    
    # Plot the coordinate axes of each individual frames
    for i in range(len(Rlist)):
        p = frame_pos[i]
        Rm = frame_rot[i]
        x_axis, y_axis = get_axes(p, Rm)
        plt.plot([p[0], x_axis[0]], [p[1], x_axis[1]], 'r-')
        plt.plot([p[0], y_axis[0]], [p[1], y_axis[1]], 'b-')
    
    # Plot formatting
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(r'Robot Configuration for $\vec{\theta} = $' + str([round(angle, 2) for angle in joint_angles]) + r' and $\vec{l} = $' + str(link_lengths))
    plt.grid(True)
    plt.xlim([-10, 10])
    plt.ylim([-2, 10])
    
    return plt
    
# Example usage
joint_angles = [0, 0]
plt = visualize_robot(joint_angles, [4, 2])
plt.savefig('image1.png')

joint_angles = [-np.pi/4, -np.pi/2]
plt = visualize_robot(joint_angles, [4, 2])
plt.savefig('image2.png')

joint_angles = [np.pi/8, -2*np.pi/3]
plt = visualize_robot(joint_angles, [2, 3])
plt.savefig('image3.png')