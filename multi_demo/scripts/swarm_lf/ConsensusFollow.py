from math import exp
import numpy as np
import scipy 

from DesirePath import DesirePath


class ConsensusFollow:
    # 二维
    converge_k = 500.   # 收敛参数：越大收敛到期望轨迹越慢，但相应的控制约平缓
    consensus_k = 0.1    # 一致性增益：越大收敛到期望编队越快
    def __init__(self, start_pos, desire_path, L, self_id, offset, dt=0.1) -> None:
        # 无人机状态
        self.status = np.append(np.array(start_pos), [10., 0.])
        self.ref_status = np.append(np.array(start_pos) - offset, [1., 0.])
        # 轨迹
        self.path = desire_path
        self.dt = dt
        # 拉普拉斯矩阵
        self.L = np.copy(L)
        self.id = self_id
        # 编队参数
        self.offset = np.copy(offset)
        # 轨迹跟踪相关
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

        # 状态储存
        self.status_record = np.array([self.status])
        self.err_record = np.array([[0., 0., 0., 0.]])

    def DesireStatusCal(self, ref_r):
        p_now = self.status[[0, 1]]
        v_now = self.status[[2, 3]]
        
        ref_psi_sin = v_now[1] / np.linalg.norm(v_now)
        ref_psi_cos = v_now[0] / np.linalg.norm(v_now)
        trans_m = np.array([[ref_psi_cos, -ref_psi_sin],
                            [ref_psi_sin, ref_psi_cos]])
        # 计算参考位置
        ref_p = np.reshape(p_now, [2]) - trans_m @ self.offset
        # 计算参考位置期望状态
        min_p, min_dis, desire_v, dis_to_end = self.path.NearestP(ref_p)
        
        desire_psi_sin = desire_v[1] / np.linalg.norm(desire_v)
        desire_psi_cos = desire_v[0] / np.linalg.norm(desire_v)
        trans_m_2 = np.array([[desire_psi_cos, -desire_psi_sin],
                              [desire_psi_sin, desire_psi_cos]])
        # 计算期望状态
        desire_p = np.reshape(min_p, [2]) + trans_m_2 @ self.offset

        if abs(ref_r) < 0.1**10:
            desire_v = np.copy(desire_v)
        else:
            omega = np.array([0., 0., np.linalg.norm(desire_v) / ref_r])
            offset_o = trans_m @ self.offset
            offset_o = np.array([offset_o[0], offset_o[1], 0.])
            v_add = np.cross(omega, offset_o)
            desire_v = desire_v + v_add[[0, 1]]
        return desire_p, desire_v

    def Acal(self, all_agent_pos, ref_r=0.):
        # 期望轨迹给出的参考状态
        desire_p, desire_v = self.DesireStatusCal(ref_r)
        # 收敛速度
        uav_pos = self.ref_status[0: 2]
        converge_vec = desire_p - uav_pos
        converge_v = converge_vec / self.converge_k
        # 一致性速度
        consensus_v = -self.L[self.id, :] @ all_agent_pos * self.consensus_k
        if np.linalg.norm(consensus_v) > 4.:
            consensus_v = consensus_v / np.linalg.norm(consensus_v) * 4.

        desire = desire_v + converge_v
        desire = desire / np.linalg.norm(desire) * np.linalg.norm(desire_v)
        desire = desire + consensus_v

        desire_status = np.append(desire_p, desire)
        err_status = self.status - desire_status

        # LQR计算加速度方向
        u = self.C * self.K @ err_status
        # 加速度限幅
        if np.linalg.norm(u) > 5:
            u = u / np.linalg.norm(u) * 5.
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
        # 参考点更新
        p_now = self.status[[0, 1]]
        v_now = self.status[[2, 3]]
        ref_psi_sin = v_now[1] / np.linalg.norm(v_now)
        ref_psi_cos = v_now[0] / np.linalg.norm(v_now)
        trans_m = np.array([[ref_psi_cos, -ref_psi_sin],
                            [ref_psi_sin, ref_psi_cos]])
        ref_p = np.reshape(p_now, [2]) - trans_m @ self.offset
        self.ref_status[[0, 1]] = ref_p
        self.ref_status[[2, 3]] = v_now

        

        self.status_record = np.append(self.status_record, [self.status], axis=0)

    def GetErr(self, desire_status):
        err = self.status - desire_status - self.offset
        return err

    @ staticmethod
    def RiccatiEqSolve(A, B, Q, R):
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        return P


def SingleTest():
    # 期望轨迹定义
    points = np.array([[0., 0.], [500., 0.], [500., 500.], [0., 500.]])
    desire_v = 12.
    path = DesirePath(points, desire_v, loop_flag=True)

    # 队形定义
    formation = np.array([[-10., -10.],
                          [-10., 10.],
                          [10., 10.],
                          [10., -10.]])
    # 起始位置
    start_pos = np.array([[0., 0.],
                          [10., 0.],
                          [20., 0.],
                          [30., 0.]])
    # 拉普拉斯矩阵
    L = np.array([[2, -1, 0, -1],
                  [-1, 2, -1, 0],
                  [0, -1, 2, -1],
                  [-1, 0, -1, 2]])
    # 无人机实例化
    dt = 0.1
    uavs = []
    for i in range(4):
        uavs.append(ConsensusFollow(start_pos[i, :], path, L, i, formation[i, :], dt))

    step_amount = 1500
    for steps in range(step_amount):
        all_p = np.zeros([4, 2])
        for i in range(4):
            all_p[i, :] = uavs[i].ref_status[[0, 1]]
        for i in range(4):
            u = uavs[i].Acal(all_p)
            uavs[i].StatusUpdate(u)
            # print(uav.AngleCal(u))

    import plotly.graph_objects as go
    # 轨迹绘图
    plt = go.Figure()
    # 参考轨迹绘制
    plt.add_trace(go.Scatter(x=path.points[:, 0],
                             y=path.points[:, 1],
                             mode='lines',
                             line=dict(width=1., dash='solid', color='blue')))
    # 无人机轨迹绘制
    for i in range(4):
        x = uavs[i].status_record[:, 0]
        y = uavs[i].status_record[:, 1]

        plt.add_trace(go.Scatter(x=x,
                                 y=y,
                                 mode='lines',
                                 line=dict(width=1.5, dash='solid')))
    
    # 无人机编队绘制
    plt_index = np.ceil(np.linspace(0, step_amount, 20))
    for index in plt_index:
        formation_line = np.zeros([5, 2])
        for i in range(4):
            formation_line[i, :] = uavs[i].status_record[int(index), [0, 1]]
        formation_line[-1, :] = formation_line[0, :]
        plt.add_trace(go.Scatter(x=formation_line[:, 0],
                                 y=formation_line[:, 1],
                                 mode='lines',
                                 line=dict(width=2, dash='solid', color='red')))

    plt.show()


if __name__ == "__main__":
    # UAVTest()
    # ErrTest()
    SingleTest()


    
