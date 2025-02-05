#!/usr/bin/env python3
import numpy as np
import plotly.graph_objects as go


class lissajous:
    def __init__(self):
        # 李萨如曲线相关参数
        self.amplitude = np.array([225., 225., -20.])
        self.phase = np.array([0.001, 0.002, 0.002])
        # self.phase = np.array([0.01, 0.02, 0.02])
        self.offset = np.array([0., np.pi/2., 0.])
        self.add = np.array([0., 0., 80.])

    def GetPhi(self, pos):
        # 计算phi
        phi = np.zeros([3])
        for i in range(3):
            phi[i] = pos[i] - self.amplitude[i] * np.cos(self.phase[i] * pos[-1] + self.offset[i]) - self.add[i]
        return phi
    
    def GetDPhi(self, pos):
        # 计算误差梯度
        d_phi = np.eye(3)
        d_phi_add = np.zeros([3, 1])
        for i in range(3):
            d_phi_add[i, 0] = self.phase[i] * (self.amplitude[0]) * np.sin(self.phase[i] * pos[-1] + self.offset[i])
        d_phi = np.append(d_phi, d_phi_add, axis=1)
        return d_phi
    
    def GetSamplePath(self):
        angle_list = np.linspace(0., 5000 * np.pi, 300)
        x = self.amplitude[0] * np.cos(self.phase[0] * angle_list + self.offset[0])
        y = self.amplitude[1] * np.cos(self.phase[1] * angle_list + self.offset[1])
        z = self.amplitude[2] * np.cos(self.phase[2] * angle_list + self.offset[2]) + self.add[2]
        return x, y, z


class GVF:
    def __init__(self):
        self.path = lissajous()
        self.k = np.eye(3) * 0.001
        self.k = np.array([[0.001,0,0], [0,0.001,0], [0,0,0.05]])

    def GetGuidVec(self, pos):
        phi = self.path.GetPhi(pos)
        print('phi',phi)
        d_phi = self.path.GetDPhi(pos)

        converge_vec = -np.reshape(phi, [1, -1]) @ self.k @ d_phi
        transport_vec = -np.append(d_phi[:, -1], [-1], axis=0)

        converge_vec = np.reshape(converge_vec, [-1])
        transport_vec = np.reshape(transport_vec, [-1])
        # print('transport_vec',transport_vec)
        # print('converge_vec',converge_vec)
        guid_vec = converge_vec + transport_vec
        guid_vec /= np.linalg.norm(guid_vec[0:3])
        return guid_vec


if __name__ == "__main__":
    pos_now = np.array([0.,0.,0.,0.])
    gvf = GVF()

    status_record = np.array([pos_now])

    t_step = 0.1
    v = 25
    sim_time = 1000

    # gvf.path.GetDPhi(pos_now)
    # gvf.path.GetPhi(pos_now)

    for i in np.linspace(0, 1000, int(sim_time / t_step)):
        # print(i)
        guid_vec = gvf.GetGuidVec(pos_now)
        print(i,guid_vec)
        # guid_vec[0:3] *= v
        pos_now +=  v * guid_vec * t_step
        status_record = np.append(status_record, [pos_now], axis=0)

    plt = go.Figure()
    # ------绘制轨迹-------
    plt.add_trace(go.Scatter3d(x=status_record[:, 0],
                               y=status_record[:, 1],
                               z=status_record[:, 2],
                               mode='lines',
                               line=dict(width=2, dash='solid', color='blue')))
    
    # ------绘制参考路径-------
    sample_x, sample_y, sample_z = gvf.path.GetSamplePath()
    plt.add_trace(go.Scatter3d(x=sample_x,
                               y=sample_y,
                               z=sample_z,
                               mode='lines',
                               line=dict(width=1, dash='solid', color='red')))
    
    plt.update_layout(scene=dict(aspectmode='data'))
    plt.show()
