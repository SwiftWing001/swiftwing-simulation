from math import exp
import numpy as np
import scipy 

from DesirePath import DesirePath


class PathFollow:
    # 二维
    converge_k = 500.   # 收敛参数：越大收敛越慢，但相应的控制约平缓
    def __init__(self, start_pos, desire_path, dt=0.1) -> None:
        # 无人机状态
        self.status = np.append(np.array(start_pos), [10., 0.])
        # 是否使用误差
        # self.err_flag = err_flag    # 是否使用误差
        # 轨迹
        self.path = desire_path
        self.dt = dt
        # 编队控制相关参数 ============
        self.A = np.zeros([4, 4])
        self.A[0, 2] = 1.
        self.A[1, -1] = 1.

        self.B = np.zeros([4, 2])
        self.B[2, 0] = 1.
        self.B[3, 1] = 1.

        self.R = 200. * np.eye(2)
        self.Q = 10. * np.eye(4)

        self.P = self.RiccatiEqSolve(self.A, self.B, self.Q, self.R)
        self.C = 1.

        self.K = -np.linalg.inv(self.R) @ self.B.T  @ self.P

        # 位置未校正的时间
        self.last_correction_time = 0.
        # 状态储存
        self.status_record = np.array([self.status])
        self.err_record = np.array([[0., 0., 0., 0.]])

    def Acal(self):
        # LQR计算加速度方向
        uav_pos = self.status[0: 2]
        min_p, min_dis, desire_v, dis_to_end = self.path.NearestP(uav_pos)
        print("desire_v: ", desire_v)

        converge_vec = min_p - uav_pos
        converge_v = converge_vec / self.converge_k
        desire = desire_v + converge_v
        desire = desire / np.linalg.norm(desire) * self.path.desire_v
        desire_status = np.append(min_p, desire)
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

    def GetErr(self, desire_status):
        err = self.status - desire_status - self.offset
        return err

    @ staticmethod
    def RiccatiEqSolve(A, B, Q, R):
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        return P


def SingleTest():
    # points = np.array([[0., 0.], [100., 500.], [800., 500.], [1200., 800.]])
    points = np.array([[0., 0.], [500., 0.], [500., 500.], [0., 500.]])
    # points = np.array([[0., 0.], [1000., 500.]])
    desire_v = 12.
    path = DesirePath(points, desire_v, loop_flag=False)
    start_pos = np.array([0., 0.])

    step_amount = 2000
    dt = 0.1

    uav = PathFollow(start_pos, path, dt)

    for i in range(step_amount):
        u = uav.Acal()
        uav.StatusUpdate(u)
        # print(uav.AngleCal(u))

    import plotly.graph_objects as go
    # 轨迹绘图
    plt = go.Figure()
    plt.add_trace(go.Scatter(x=uav.path.points[:, 0],
                             y=uav.path.points[:, 1],
                             mode='lines',
                             line=dict(width=1., dash='solid', color='blue')))

    x = uav.status_record[:, 0]
    y = uav.status_record[:, 1]

    plt.add_trace(go.Scatter(x=x,
                             y=y,
                             mode='lines',
                             line=dict(width=1.5, dash='solid')))


    plt.show()

    # plt2 = go.Figure()

    # x = np.linalg.norm(uav.err_record[:, 0:3], axis=1)
    # x = np.squeeze(x)
    # print(x)
    # plt2.add_trace(go.Scatter(y=x, mode='lines',
    #                           line=dict(width=1, dash='solid')))
    # plt2.show()


if __name__ == "__main__":
    # UAVTest()
    # ErrTest()
    SingleTest()


    
