import numpy as np
import matplotlib.pyplot as plt
import math

def main():
    curr_yaw_degrees = 0

    # v1 = np.array([0,1]) # heading vector
    v1 = np.array([np.cos(np.radians(curr_yaw_degrees)), np.sin(np.radians(curr_yaw_degrees))])
    v2 = np.array([2,-1]) # pos to target vector

    fig, ax = plt.subplots()
    ax.quiver(0,0, v1[0], v1[1], angles='xy', scale_units='xy', scale=1, color = 'r')
    ax.quiver(0,0, v2[0], v2[1], angles='xy', scale_units='xy', scale=1, color = 'b')
    
    ax.set_xlim([-3,3])
    ax.set_ylim([-3,3])
    
    matrix = np.vstack((v1,v2))
    print(matrix)
    dot = np.dot(v1,v2)
    det = np.linalg.det(matrix)
    alpha = np.arctan2(det, dot)

    print(np.degrees(alpha))


    dist_to_target = np.linalg.norm(v2)
    target_steering = np.arctan2(2 * 1.75 * np.sin(alpha), dist_to_target)
    
    steer_vector = math.radians(curr_yaw_degrees) + np.array([np.cos(target_steering), np.sin(target_steering)])

    ax.quiver(0,0, steer_vector[0], steer_vector[1], angles='xy', scale_units='xy', scale=1, color = 'g')

    # print(np.degrees(target_steering))

    plt.plot()
    plt.grid()  
    plt.show()

if __name__ == "__main__":
    main()