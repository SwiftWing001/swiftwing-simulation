import numpy as np
import scipy


class Follower:
    converge_k = 500.

    # 二维
    def __init__(self, start_pos, offset, dt=0.1) -> None:
        # 无人机状态
        self.status = np.append(np.array(start_pos), [10., 0.])
        # 是否使用误差
        self.offset = np.reshape(offset, [2])

        self.dt = dt
        # 编队控制相关参数 ============
        self.A = np.zeros([4, 4])
        self.A[0, 2] = 1.
        self.A[1, -1] = 1.

        self.B = np.zeros([4, 2])
        self.B[2, 0] = 1.
        self.B[3, 1] = 1.

        self.R = 100. * np.eye(2)
        self.Q = 10. * np.eye(4)

        self.P = self.RiccatiEqSolve(self.A, self.B, self.Q, self.R)
        self.C = 1.

        self.K = -np.linalg.inv(self.R) @ self.B.T  @ self.P

        # 位置未校正的时间
        self.last_correction_time = 0.
        # 状态储存
        self.status_record = np.array([self.status])
        self.err_record = np.array([[0., 0., 0., 0.]])

    def DesireStatusCal(self, leader_p, leader_v, leader_r):
        # leader_r 为领航者转弯半径，=0时直线飞行
        leader_psi_sin = leader_v[1] / np.linalg.norm(leader_v)
        leader_psi_cos = leader_v[0] / np.linalg.norm(leader_v)
        trans_m = np.array([[leader_psi_cos, -leader_psi_sin],
                            [leader_psi_sin, leader_psi_cos]])
        desire_p = np.reshape(leader_p, [2]) + trans_m @ self.offset

        leader_v = np.reshape(leader_v, [2])
        if abs(leader_r) < 0.1**10:
            desire_v = np.copy(leader_v)
        else:
            omega = np.array([0., 0., np.linalg.norm(leader_v) / leader_r])
            v = np.array([leader_v[0], leader_v[1], 0.])
            v_add = np.cross(omega, v)
            desire_v = leader_v + v_add[[0, 1]]
            # print("leader_v: ", leader_v)
            # print("add_v: ", v_add)
            # print("omega: ", omega)
            # print("leader_r: ", leader_r)
        return desire_p, desire_v

    def Acal(self, leader_p, leader_v, leader_r=0.):
        # leader_r = 0.
        desire_p, desire_v = self.DesireStatusCal(leader_p, leader_v, leader_r)
        # print("follow------: desire_p: ", desire_p, "     desire_v: ", desire_v)

        # LQR计算加速度方向
        uav_pos = self.status[0: 2]
        converge_vec = desire_p - uav_pos
        converge_v = converge_vec / self.converge_k
        desire = desire_v + converge_v
        desire = desire / np.linalg.norm(desire) * np.linalg.norm(desire_v)
        desire_status = np.append(desire_p, desire)
        err_status = self.status - desire_status

        u = self.C * self.K @ err_status
        # 加速度限幅
        if np.linalg.norm(u) > 10:
            u = u / np.linalg.norm(u) * 10.
        # 最小、最大速度限制
        v = self.status[[2, 3]]
        a = np.copy(np.squeeze(u))
        if v @ a < 0 and np.linalg.norm(v) < 10.:
            v_unit = v / np.linalg.norm(v)
            a_t = np.array([a @ v_unit * v_unit])
            u = u - np.squeeze(a_t)
        elif v @ a > 0 and np.linalg.norm(v) > 50.:
            v_unit = v / np.linalg.norm(v)
            a_t = np.array([a @ v_unit * v_unit])
            u = u - np.squeeze(a_t)
        u = np.squeeze(u)
        self.err_record = np.append(self.err_record, [err_status], axis=0)

        return u
    
    def AngleCal(self, a_vec):
        # 几何计算期望姿态
        v_vec = self.status[[2, 3]]
        a = a_vec @ v_vec / np.linalg.norm(v_vec)

        a_t = a_vec - a_vec @ v_vec * v_vec / (v_vec @ v_vec)
        g = 9.8
        phi = np.arctan(np.linalg.norm(a_t) / g)
        
        # ===计算phi的符号===
        temp = np.squeeze(np.array([[0, -1], [1, 0]]) @ np.reshape(v_vec, [-1, 1]))
        if np.linalg.norm(temp - a_vec) < np.linalg.norm(temp + a_vec):
            sig = 1.
        else:
            sig = -1.
        return a, phi * sig

    def StatusUpdate(self, u):
        dt = self.dt
        # 更新无人机位置，质点模型
        status = np.reshape(self.status, [4, 1])
        u = np.reshape(u, [2, 1])
        temp = np.squeeze((self.A @ status + self.B @ u) * dt)
        self.status += temp
        self.status_record = np.append(self.status_record, [self.status], axis=0)

    @ staticmethod
    def RiccatiEqSolve(A, B, Q, R):
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        return P
