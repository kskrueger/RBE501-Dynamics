import numpy as np


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def twist2ht(S, theta):
    w = S[:3]
    w_ss = skew(w)
    v = S[3:, None]
    t = theta
    term2 = np.eye(3) * t + (1 - np.cos(t)) * w_ss + (t - np.sin(t)) * (w_ss @ w_ss)
    T = np.vstack((np.hstack((axisangle2rot(w, t), (term2 @ v))),
                   [0, 0, 0, 1]))

    return T


def axisangle2rot(omega, theta):
    omega_ss = skew(omega)
    R = np.eye(3) + np.sin(theta) * omega_ss + (1 - np.cos(theta)) * (omega_ss @ omega_ss)
    return R


def calc_fkine_space(S, M, q):
    T = np.eye(4)

    for i in range(S.shape[1]):
        s = S[:, i]
        t = q[i]

        twist = twist2ht(s, t)
        T = T @ twist

    T = T @ M

    return T


def adjoint(twist_inA, T_AB):
    R = T_AB[:3, :3]
    p = T_AB[:3, 3]
    pR = skew(p) @ R

    AdT = np.vstack((np.hstack((R, np.zeros((3, 3)))),
                     np.hstack((pR, R))))

    twist_inB = AdT @ twist_inA.T

    return twist_inB


def jacob0(S, q):
    Js = np.zeros((6, S.shape[1]))
    T = np.eye(4)

    for i in range(S.shape[1]):
        s = S[:, i]
        T = T @ twist2ht(s, q[i])

        Js[:, i] = adjoint(s, T)

    return Js


def jacob_a(S, M, q):
    J_b = jacob_e(S, M, q)
    T = calc_fkine_space(S, M, q)
    R = T[:3, :3]
    J_a = R @ J_b[3:, :]
    return J_a


def jacob_e(S, M, q):
    J = np.zeros((6, S.shape[1]))
    T = np.eye(4)
    for i in reversed(range(S.shape[1])):
        s = S[:, i]
        T = T @ np.linalg.inv(twist2ht(s, q[i]))
        twist_inB = adjoint(s, T)
        J[:, i] = twist_inB
    return J


def calc_ik(get_poses_f, get_joint_variables_f, S, M, targetPose):
    currentQ = get_joint_variables_f() #(robot)
    currentPose = get_poses_f() #(robot)

    while np.linalg.norm(targetPose - currentPose) > 0.001:
        J = jacob0(S, currentQ.T)
        # J = jacob_a(S, M, currentQ.T)
        deltaQ = np.linalg.pinv(J) @ (targetPose - currentPose)
        currentQ = currentQ + deltaQ.T

        T = calc_fkine_space(S, M, currentQ.T)
        currentPose = MatrixLog6(T)
        currentPose = np.array([[currentPose[2, 1], currentPose[0, 2], currentPose[1, 0], *currentPose[:3, 3]]]).T
        print("currQ", currentQ)
        print("poseError", targetPose - currentPose)

    return currentQ


def MatrixLog3(R):
    acosinput = (np.trace(R) - 1) / 2
    if acosinput >= 1:
        so3mat = np.zeros((3, 3))
    elif acosinput <= -1:
        if not np.all((1 + R[2, 2]) < .001):
            omg = (1 / np.sqrt(2 * (1 + R[2, 2]))) @ (np.vstack([R[0, 2], R[1, 2], (1 + R[2, 2])]))
        elif not np.all((1 + R[2, 2]) < .001):
            omg = (1 / np.sqrt(2 * (1 + R[1, 1]))) @ (np.vstack([R[0, 1], (1 + R[1, 1]), R[2, 1]]))
        else:
            omg = (1 / np.sqrt(2 * (1 + R[0, 0]))) @ (np.vstack([(1 + R[0, 0]), R[1, 0], R[2, 0]]))
        so3mat = skew(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        so3mat = theta * (1 / (2 * np.sin(theta))) * (R - R.T)

    return so3mat


def MatrixLog6(T):
    R = T[:3, :3]
    p = T[:3, 3:]
    omg_mat = MatrixLog3(R)
    if (omg_mat == np.zeros((3, 3))).all():
        exp_mat = np.vstack([np.hstack([np.zeros((3, 3)), T[:3, 3:]]),
                             [0, 0, 0, 0]])
    else:
        theta = np.arccos((np.trace(R) - 1) / 2.0)
        a = (np.eye(3) - omg_mat / 2.0 + (1.0 / theta - (1.0 / np.tan(theta / 2.0)) / 2.0) * (
                omg_mat @ omg_mat) / theta) @ p
        exp_mat = np.vstack([np.hstack([omg_mat, a]),
                             [0, 0, 0, 0]])

    return exp_mat


def path_line(start, goal):
    # start [x, y, z]
    # goal [x, y, z]
    N = 100
    path = []
    distance_x = (goal[0] - start[0])
    distance_y = (goal[1] - start[1])
    distance_z = (goal[2] - start[2])
    for i in range(N + 1):
        dX = start[0] + i / N * distance_x
        dY = start[1] + i / N * distance_y
        dZ = start[2] + i / N * distance_z
        path.append([start[0] + dX, start[1] + dY, start[2] + dZ])

    return path


# Tests
if __name__ == '__main__':
    S = np.array([[0, 0, 1.0000, 0, 0, 0],
                  [1.0000, 0, 0, 0, 0.4000, 0],
                  [1.0000, 0, 0, 0, 0.4000, -0.3000]]).T

    M = np.array([[0, 1.0000, 0, 0],
                  [0, 0, -1.0000, 0.3000],
                  [-1.0000, 0, 0, 0.2000],
                  [0, 0, 0, 1.0000]])

    # j = jacob0(np.array([[1,2,3,4,5,6], [7,8,9,10,11,12]]).T, np.array([1,2]))

    q = np.array([1, 2, 3])
    a = calc_fkine_space(S, M, q)

    # UR5 test
    # arm lengths
    d1 = 0.089159  # [m]
    d4 = 0.10915  # [m]
    d5 = 0.09465  # [m]
    d6 = 0.0823  # [m]
    a2 = 0.425  # [m]
    a3 = 0.39225  # [m]
    a4 = 0
    a6 = 0

    # Twists and Home config
    S = np.transpose(np.array([[0, 0, 1, 0, 0, 0],
                               [0, -1, 0, d1, 0, 0],
                               [0, -1, 0, d1, 0, -a2],
                               [0, -1, 0, d1, 0, -(a2 + a3)],
                               [0, 0, -1, 0, a2 + a3 + a4, 0],
                               [0, -1, 0, d1 - d5, 0, -(a2 + a3 + a4)]]))

    M = np.array([[1, 0, 0, a2 + a3 + a4 + a6], [0, 0, -1, 0], [0, 1, 0, d1 - d5 - d6], [0, 0, 0, 1]])

    q = np.array([0, 0, 0, 0, 0, 0])
    ur5_fkine = calc_fkine_space(S, M, q)
    # print("ur5_fkine", ur5_fkine)

    # Test Inverse Kinematics without pyBullet first

    q0 = np.array([0, 0, 0.1, 0, 0, 0])
    T = calc_fkine_space(S, M, q0)
    targetPose = MatrixLog6(T)
    targetPose = np.array([[targetPose[2, 1], targetPose[0, 2], targetPose[1, 0], *targetPose[:3, 3]]]).T

    print("targetPose", targetPose)


    def get_poses_f():
        home_pose = MatrixLog6(M)
        return np.array([[home_pose[2, 1], home_pose[0, 2], home_pose[1, 0], *home_pose[:3, 3]]]).T

    def get_joints_f():
        return np.array([0, 0, 0, 0, 0, 0])

    ik_q = calc_ik(get_poses_f, get_joints_f, S, M, targetPose)
    print("ik_q", ik_q)

    print()