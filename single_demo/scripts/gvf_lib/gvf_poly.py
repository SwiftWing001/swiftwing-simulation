#!/usr/bin/env python3
from math import isnan
from matplotlib.cbook import flatten
import numpy as np

import os
import sys
dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/gvf_lib"
sys.path.insert(0, dir_mylib)
print(dir_mylib)



class GVF:
    # mc_flag : mixed convergence 混合收敛
    # ag_flag : adaptive gain 自适应增益
    def __init__(self, path, ori=0, k=1, mc_flag=0) -> None:
        self.mc_flag = mc_flag

        self.ori = ori  # 正反两方向

        self.path = path
        self.dimension = self.path.dimension
        # k = 50.
        self.k = np.eye(self.dimension + 1) * k
        self.k[-1, -1] /= 1.
        self.m = np.eye(self.dimension + 1)

    def GetError(self, pos, t):
        phi_list = self.path.GetPhi(pos, t)
        e_list = phi_list**2 / 2
        E = np.sum(e_list)
        return E
            
    def GetConvergeVec(self, pos, t):
        e_jacobian = self.path.GetEJacobian(pos, t)
        e_hessian = self.path.GetEHessian(pos, t)
        # print("e_hess: ", e_hessian)

        m = np.eye(self.dimension + 1)
        # 混合收敛策略
        if self.mc_flag:
            # eta = self.AdjustFun(e_hessian)
            # m_len = self.HesLenCal(e_hessian, eta)
            # m_temp = e_hessian + eta * np.eye(self.dimension + 1)
            # m = m_temp / np.linalg.norm(np.linalg.eigvals(m_temp)) * m_len
            # m = e_hessian
            # if m[-1, -1] < 0.1:
            #     m[-1, -1] = 0.1
            m = e_hessian
            eta = self.AdjustFun(np.linalg.eigvals(m))
            m = m + eta * np.eye(self.dimension + 1)
        inv_m = np.linalg.inv(m)
        conver_vec = self.k @ (inv_m @ np.array([e_jacobian]).T)
        # print("inv_m:", inv_m)
        # print("e_jacobian", e_jacobian)
        # print("conver_vec", conver_vec)
        if e_jacobian[1] > 10**10:
            pass
        return np.squeeze(conver_vec)
         
    def GetTransVec(self, pos, t):
        d_phi = self.path.GetPhiJacobian(pos, t)
        # 将tau作为交换到第一列计算楔积，以免因为维度变化影响方向
        trans_seg = [-1] + [i for i in range(self.dimension)]
        d_phi = d_phi[:, trans_seg]
        # 计算楔积
        wedge_product = np.zeros([self.dimension + 1])
        for i in range(self.dimension + 1):
            upper = np.zeros([1, self.dimension + 1])
            upper[0, i] = 1
            wedge_product_matrix = np.append(upper, d_phi, axis=0)
            wedge_i = np.linalg.det(wedge_product_matrix)
            wedge_product[i] = wedge_i
        trans_seg = [i+1 for i in range(self.dimension)] + [0]
        wedge_product = wedge_product[trans_seg]
        # print("d_phi: ", d_phi)
        # print("wedge_product: ", wedge_product)
        return wedge_product
        
    def GetVec(self, pos, t):
        trans_vec = self.GetTransVec(pos, t) 
        con_vec_ori = self.GetConvergeVec(pos, t)
        conve_vec = con_vec_ori - con_vec_ori @ trans_vec * trans_vec /\
            np.linalg.norm(trans_vec)**2
        vec = (-1)**self.ori * trans_vec - conve_vec
        vec /= np.linalg.norm(vec[0:self.dimension])
        return vec
    
    def GetVecField(self, t):
        x = np.linspace(-50, 150, 20)
        y = np.linspace(-50, 150, 20)
        x, y = np.meshgrid(x, y)

        desire_p = self.path.GetDesireP(t)
        z_0 = desire_p[-1]
        z = z_0 + np.zeros(x.shape)
        x = x.flatten()
        y = y.flatten()
        z = z.flatten()

        u_ = np.zeros([0])
        v_ = np.zeros([0])
        w_ = np.zeros([0])
        for i in range(len(x)):
            u, v, w, t = self.GetVec([x[i], y[i], z_0], t)
            u_ = np.append(u_, u)
            v_ = np.append(v_, v)
            w_ = np.append(w_, w)
        return x, y, z, u_, v_, w_

    @ staticmethod  # 用于计算海瑟矩阵的原始长度
    def HesLenCal(hes_matrix, eta, method=2):
        if method == 0:     # 原始海瑟矩阵的长度
            hes_len = np.linalg.norm(np.linalg.eigvals(hes_matrix))
        elif method == 1:   # 第一种根据eta和原始长度计算方式（长度不一定在原始长度和1之间）
            origin_len = np.linalg.norm(np.linalg.eigvals(hes_matrix))
            # print("origin_hes_len:", origin_len)
            hes_len = (origin_len + eta) / np.log(np.e - 1 + np.e**eta)
        elif method == 2:   # 第二种根据eta和原始长度计算方式（长度一定在原始长度和1之间）
            origin_len = np.linalg.norm(np.linalg.eigvals(hes_matrix))
            # print("origin_hes_len:", origin_len)
            if eta > 20:
                hes_len = 1
            else:
                hes_len = (origin_len - 1) / np.e**eta + 1
        return hes_len
    
    @staticmethod   # 计算eta，用于调整hes矩阵和I的混合值
    def AdjustFun(eigvals):
        min_eigvals = np.min(eigvals)
        det = np.prod(eigvals)
        
        tolerant = 0.01

        if min_eigvals > tolerant:
            adjust = 0
        else:
            adjust = (tolerant - min_eigvals)

        # print("-------------- eta:", adjust, "----min_eigvals:", min_eigvals, "---det", det)
        return float(adjust)


class circleGVF:
    def __init__(self, r, center=[0., 0., 0.], ori=0, k=1.):
        self.r = float(r)
        self.center = np.squeeze(np.array(center))
        self.ori = ori
        self.k = k

    def GetPhi(self, pos):
        pose = np.array(pos) - self.center
        phi1 = pose[2]
        phi2 = np.linalg.norm(pose[0:2]) - self.r
        # print("x", x, "y", y, "z", z, "phi2: ", phi2)
        return [phi1, phi2]

    def GetJacobin(self, pos):
        pose = np.array(pos) - self.center
        length = np.linalg.norm(pose[0:2])
        if length <= 10.**-10:
            length = 0.00001
        jacobin = np.array([[0., 0., 1.], [2 * pose[0] / length, 2 * pose[1] / length, 0.]])
        return jacobin

    def GetConvergeVec(self, pos):
        phi = self.GetPhi(pos)
        jacobin = self.GetJacobin(pos)
        con_vec = phi[0] * jacobin[0, :] + phi[1] * jacobin[1, :]
        # print("con_vec: ", con_vec)
        return con_vec

    def GetTransVec(self, pos):
        jacobin =self.GetJacobin(pos)
        trans_vec = np.cross(jacobin[0], jacobin[1])
        # print("trans_vec: ", trans_vec)
        # trans_vec = np.array([0., 0., 0.])
        return trans_vec

    def GetVec(self, pos):
        trans_vec = self.GetTransVec(pos)
        conve_vec = self.GetConvergeVec(pos)
        vec = trans_vec - self.k * conve_vec
        if vec[-1] > np.linalg.norm(vec) * 0.3:
            vec[-1] = np.linalg.norm(vec) * 0.3
        return vec / np.linalg.norm(vec)

if __name__ == "__main__":
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    from PolynomialTraj import PolynomialTraj3Order
    points = np.array([[100., 0., 0.],
                       [300., 0., 10.],
                       [300., 50., 20.],
                       # [100., -50., 80.],
                       [300., 50., 0.],
                       [100., 100., 10.]])
    # points = np.array([[0., 40.],
    #                    [300., 0.],
    #                    [300., 100.],
    #                    [100., 200.]])
    # t_list = np.array([0., 5., 10., 15.])
    dimension = 3
    traj = PolynomialTraj3Order(dimension, points, loop_flag=0)
    gvf = GVF(traj, k=5, mc_flag=1)

    vel = 20.
    t_step = 0.1
    t_amount = 500
    t_now = 0.
    plt = go.Figure()
    # 期望轨迹
    sample_traj = traj.GetSampleTraj()
    plt.add_trace(go.Scatter3d(x=sample_traj[:, 0],
                            y=sample_traj[:, 1],
                            z=sample_traj[:, 2],
                            mode='lines',
                            line=dict(width=3, dash='solid', color='red')))
    plt.add_trace(go.Scatter3d(x=points[:, 0],
                            y=points[:, 1],
                            z=points[:, 2],
                            mode='markers',
                            marker=dict(size=4)))
    for i in range(3):
        t_now = 0.
        print(i)
        # pos_now = np.array([100., 100., 30.]) 
        pos_now = np.random.random(3) * 1
        pos_record = np.array([np.append(pos_now, [t_now])])
        print("-----------------")
        for i in range(t_amount):
            print("t_now: ", t_now)
            vec = gvf.GetVec(pos_now, t_now)
            vec_go = vel * vec[0:traj.dimension] * t_step
            t_now += vec[-1]
            pos_now += vec_go

            pos_record = np.append(pos_record,
                                np.array([np.append(pos_now, [t_now])]), axis=0)


        plt2 = go.Figure()
        # t的变化
        plt2.add_trace(go.Scatter(y=pos_record[:, -1],
                                mode='lines',
                                line=dict(width=3, dash='solid', color='blue')))
        plt2.show()


        # 飞行轨迹
        plt.add_trace(go.Scatter3d(x=pos_record[:, 0],
                                y=pos_record[:, 1],
                                z=pos_record[:, 2],
                                mode='lines',
                                line=dict(width=3, dash='solid', color='blue')))
        
        # # 绘制向量场
        # x, y, z, u, v, w = gvf.GetVecField(25.)
        # plt.add_trace(go.Cone(x=x, y=y, z=z, u=u, v=v, w=w,
        #                       sizemode="absolute", sizeref=1))
        # x, y, z, u, v, w = gvf.GetVecField(20.)
        # plt.add_trace(go.Cone(x=x, y=y, z=z, u=u, v=v, w=w,
        #                       sizemode="absolute", sizeref=1))
        # x, y, z, u, v, w = gvf.GetVecField(15.)
        # plt.add_trace(go.Cone(x=x, y=y, z=z, u=u, v=v, w=w,
        #                       sizemode="absolute", sizeref=1))
    plt.show()

