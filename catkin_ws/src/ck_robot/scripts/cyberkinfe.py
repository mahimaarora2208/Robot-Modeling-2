import numpy as np
import matplotlib.pyplot as plt
from numpy.core.arrayprint import printoptions

def transformation_matrix(a, alpha, d, theta):
    matrix_list = []  # Will store matrix from T0_n frames
    A = np.identity(4)
    dh_table = np.array([[a[0], alpha[0], d[0], theta[0]],
                         [a[1], alpha[1], d[1], theta[1]],
                         [a[2], alpha[2], d[2], theta[2]],
                         [a[3], alpha[3], d[3], theta[3]],
                         [a[4], alpha[4], d[4], theta[4]],
                         [a[5], alpha[5], d[5], theta[5]]])
   
    for i in range(0, len(dh_table)):
        T = [[np.cos(dh_table[i, 3]), -np.sin(dh_table[i, 3]) * np.cos(dh_table[i, 1]), np.sin(dh_table[i, 3]) * np.sin(dh_table[i, 1]), dh_table[i, 0] * np.cos(dh_table[i, 3])],
             [np.sin(dh_table[i, 3]), np.cos(dh_table[i, 3]) * np.cos(dh_table[i, 1]), -np.cos(
                 dh_table[i, 3]) * np.sin(dh_table[i, 1]), dh_table[i, 0] * np.sin(dh_table[i, 3])],
             [0, np.sin(dh_table[i, 1]), np.cos(
                 dh_table[i, 1]), dh_table[i, 2]],
             [0, 0, 0, 1]]
                
        A = A @ T
        matrix_list.append(A)
        with np.printoptions(precision=2, suppress=True):
           print(A)
    return matrix_list


def jacobian_matrix(matrix_list):
    j_temp = []
    T0_6 = matrix_list[5]      # T0_6 4X4 matrix
    # d0_7 3X1 translational matrix for T0_7
    d0_6 = T0_6[0:3, 3]
    Zi_col = [[0, 0, 1]]  # Initial R0_0 value
    for i in range(0, len(matrix_list)):
        # Extracting 4th column from T0_n
        di_col = matrix_list[i][0:3, 3]
        # Linear Velocity (J_v)
        # J_v = R_i X [O_n - O_i]
        j_linear_values = np.cross(Zi_col, d0_6 - di_col)
        j_cols = np.append(j_linear_values, [Zi_col])
        j_temp.append(j_cols)
        j_final = np.array(j_temp, dtype=float).transpose() 
        # Extracting 3rd column from T0_n
        Zi_col = matrix_list[i][0:3, 0:3] @ [0, 0, 1]
    # deletes third row as third joint is fixed
    # jacobian_matrix = np.delete(j_final, 2, 1)

    return j_final


def circle_traj(radius, x_offset, y_offset, z_offset):
    x_val = []
    y_val = []
    z_val = []
    # Equally spaced angles of a circle
    s = np.linspace(0, 2 * 3.14)
    for i in s:
        x_val.append((radius * np.sin(i) * 2 * np.pi/5) + x_offset)          
        z_val.append((radius * np.cos(i) * 2 * np.pi/5) + z_offset)          
        y_val = np.ones(50) * y_offset
    return x_val, y_val, z_val


def main():
    t_0 = 0
    # theta = [0, -np.pi/6, 0, -np.pi/4, 0, -np.pi/2.4, 0]
    theta = [np.pi/2, 0, 0, 0, 0, 0, 0]
    d = [630, 0, 0, 190, 0, 0] 
    alpha = [np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2]
    a = [300, 680, 0, 0, 0, 0]

    T0_n = transformation_matrix(a, alpha, d, theta)
    # Display matrices
    print("transformation matrix(4X4 T0_6): ")
    with np.printoptions(precision=2, suppress=True):
        print(T0_n)

    # calculate Jacobian
    J = jacobian_matrix(T0_n)
    print("\n\nJacobian(6X6): ")
    with np.printoptions(precision = 3, suppress = True):
        print(J)

    # Getting the trajectory of the circle
    x, y, z = circle_traj(50, 0, 605, 680)
    # Plotting the circle
    #ax = plt.axes(projection='3d')
    #ax.plot3D(x, y, z, 'bo')
    time_step = 0.02

    # Folowing circular trajectory
    for j in range(0, len(x)):
        # The position of the end effector
        curr_d = T0_n[5][0:3, 3]
        # The required position
        req_d = np.array([[x[j]], [y[j]], [z[j]]]).transpose()
        x_dot = (req_d - curr_d) / time_step
        # Rate of change in the position
        print(np.append(x_dot, np.array([[0, 0, 0]]), axis=1))
        # Rate of change in angles
        q_dot = np.linalg.pinv(J) @ np.append(x_dot, np.array([[0, 0, 0]]), axis = 1).transpose()
        q_dot_final = q_dot.flatten()
        for k in range(0, 5):
            theta[k] = (theta[k] + q_dot_final[k] * time_step)

        T0_n = transformation_matrix(a, alpha, d, theta)
        # Calculate the new jacobian
        J = jacobian_matrix(T0_n)
        with np.printoptions(precision=2, suppress=True):
            print(T0_n)
        #ax.plot3D(curr_d[0], curr_d[1], curr_d[2], 'go')
        #plt.pause(time_step)

    #plt.show()


if __name__ == "__main__":
    main()
  # with np.printoptions(precision=2, suppress=True):
    #     print(j_trans)
