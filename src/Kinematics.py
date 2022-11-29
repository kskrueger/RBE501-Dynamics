import numpy as np


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def twist2ht(S, theta):
    # w = S[:3]
    # w_ss = skew(w)
    #
    # R = np.eye(3) + np.sin(theta) * w_ss + (1 - np.cos(theta)) * (w_ss @ w_ss)
    #
    # TopRight = (np.eye(3) * theta + (1 - np.cos(theta)) * w_ss + (theta - np.sin(theta)) * (w_ss @ w_ss))
    #
    # v = S[3:].T
    #
    # T = np.vstack((np.hstack((R, TopRight @ v)), np.array([0, 0, 0, 1])))

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
        T = T @ twist2ht(S[:, i], q[i])

        Js[:, i] = adjoint(S[:, i], T)

    return Js


def IK(robot, q):
    currentPose = robot.getPose()
    currentQ = robot.getQ()

    T = calc_fkine_space(S, M, q)
    targetPose = MatrixLog6(T)
    targetPose = np.array([[targetPose[2, 1], targetPose[0, 2], targetPose[1, 0], *targetPose[:3, 3]]]).T

    while np.linalg.norm(targetPose - currentPose) > 1e-3:
        J = jacob0(S, q)
        deltaQ = np.linalg.pinv(J) @ (targetPose - currentPose)

        currentQ = currentQ + deltaQ

        T = calc_fkine_space(S, M, currentQ)
        currentPose = MatrixLog6(T)
        currentPose = np.array([[currentPose[2, 1], currentPose[0, 2], currentPose[1, 0], *currentPose[:3, 3]]]).T

        robot.moveTargetQ(currentQ)


def calc_ik(get_poses_f, get_joint_variables_f, S, M, targetPose):
    currentPose = get_poses_f() #(robot)
    currentQ = get_joint_variables_f() #(robot)

    while np.linalg.norm(targetPose - currentPose) > 0.001:
        J = jacob0(S, currentQ)
        deltaQ = np.invert(J) @ (targetPose - currentPose)
        currentQ = currentQ + deltaQ.T

        T = calc_fkine_space(S, M, currentQ)
        currentPose = MatrixLog6(T)
        currentPose = np.vstack((currentPose[2, 1], currentPose[0, 2], currentPose[1, 0], currentPose[0, 3],
                                 currentPose[1, 3], currentPose[2, 3]))

    return currentPose


def MatrixLog3(R):
    acosinput = (np.trace(R) - 1) / 2
    if acosinput >= 1:
        so3mat = np.zeros((3, 3))
    elif acosinput <= -1:
        if ~np.isclose(1 + R[3, 3]):
            omg = (1 / np.sqrt(2 * (1 + R[3, 3]))) @ (np.vstack([R[1, 3], R[2, 3], (1 + R[3, 3])]))
        elif ~np.isclose(1 + R[2, 2]):
            omg = (1 / np.sqrt(2 * (1 + R[2, 2]))) @ (np.vstack([R[1, 2], (1 + R[2, 2]), R[3, 2]]))
        else:
            omg = (1 / np.sqrt(2 * (1 + R[1, 1]))) @ (np.vstack([(1 + R[1, 1]), R[2, 1], R[3, 1]]))
        so3mat = skew(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        so3mat = theta * (1 / (2 * np.sin(theta))) * (R - R.T)

    return so3mat


def MatrixLog6(T):
    R = T[:3, :3]
    p = T[:3, 3:]
    omg_mat = MatrixLog3(R)
    if np.equal(omg_mat, np.zeros((3, 3))).all():
        exp_mat = np.vstack([np.hstack([np.zeros((3, 3), T[:3, 4])]),
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

q0 = np.array([0, 0, 0])
T = calc_fkine_space(S, M, q0)
targetPose = MatrixLog6(T)
targetPose = np.array([[targetPose[2, 1], targetPose[0, 2], targetPose[1, 0], *targetPose[:3, 3]]]).T

print(targetPose)