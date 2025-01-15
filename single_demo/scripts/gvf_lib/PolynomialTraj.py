import numpy as np
from Polynomial import Polynomial3Order
from scipy import integrate


def generate_ordered_path_points(num_points, grid_size=5000):
    # 计算网格边长，确保网格点数大于或等于所需点数
    grid_side = int(np.ceil(np.sqrt(num_points)))
    
    # 生成一个较大的网格
    x_coords = np.linspace(0, grid_size, grid_side)
    y_coords = np.linspace(0, grid_size, grid_side)

    # 创建网格点
    xx, yy = np.meshgrid(x_coords, y_coords)
    grid_points = np.column_stack((xx.ravel(), yy.ravel()))

    # 随机打乱网格点顺序
    np.random.shuffle(grid_points)

    # 选取前num_points个点作为路径点
    path_points = grid_points[:num_points]

    # 添加z坐标
    path_points = np.column_stack((path_points, np.zeros(num_points)))
    
    return path_points

class PolynomialTraj3Order():
    def __init__(self, dimension, points, vel=[], auto_vel_flag=1,
                 loop_flag=0):
        self.v = 0.1
        self.dimension = dimension      # 曲线的维度
        self.points = np.array(points)  # 关键点
        self.loop_flag = loop_flag
        # 自动计算速度
        if auto_vel_flag:
            self.vel = self.AutoVel()
        else:
            self.vel = vel
        # 是否是闭合曲线
        if loop_flag:
            self.points = np.append(self.points, self.points[[0]], axis=0)
            self.vel = np.append(self.vel, self.vel[[0]], axis=0)
        self.t_list = self.AutoTlist()
        # 生成每个维度的多项式轨迹
        key_points = [np.append([self.points[:, i]], [self.vel[:, i]], axis=0)
                      for i in range(self.dimension)]
        polynomials = [Polynomial3Order(key_points[i].T, self.t_list)
                       for i in range(self.dimension)]
        self.polys = polynomials
        # 暂存数据，减少重复计算
        self.AdjustT_t = self.AdjustT(self.t_list[-1])
        self.pos_record = np.ones([self.dimension]) * -2.
        self.phi = np.zeros([self.dimension])
        self.phi_jacobian = np.zeros([self.dimension, self.dimension + 1])
        self.e_jacobian = np.zeros([self.dimension + 1])
        self.e_hessian = np.zeros([self.dimension + 1, self.dimension + 1])
        self.Update(self.GetDesireP(self.t_list[0]), self.t_list[0])

    def AutoVel(self):
        def GetMidVec(vec1, vec2):
            vec1_norm = np.linalg.norm(vec1)

            vec2_norm = np.linalg.norm(vec2)
            vec1 = vec1 / vec1_norm * vec2_norm
            vec2 = vec2 / vec2_norm * vec1_norm
            vec_mid = vec1 + vec2
            temp = vec_mid / np.linalg.norm(vec_mid) * self.v
            return [temp]
        vel = np.zeros([0, self.dimension])
        if self.loop_flag:
            vec1 = self.points[0] - self.points[-1]
            vec2 = self.points[1] - self.points[0]
            vec3 = self.points[-1] - self.points[-2]
            vel_start = GetMidVec(vec1, vec2)
            vel_end = GetMidVec(vec3, vec1)
        else:
            vel_start = np.zeros([1, self.dimension])
            vel_start[0, 0] = self.v * 0
            vel_end = np.zeros([1, self.dimension])
            vel_end[0, 0] = -self.v * 0
        vel = np.append(vel, vel_start, axis=0)
        for i in range(len(self.points) - 2):
            vec1 = self.points[i + 1] - self.points[i]
            vec2 = self.points[i + 2] - self.points[i + 1]
            temp = GetMidVec(vec1, vec2)
            vel = np.append(vel, temp, axis=0)
        # vel_end = np.zeros([1, self.dimension])
        # vel_end[0, 0] = -self.v * 0
        vel = np.append(vel, vel_end, axis=0)
        return vel

    def AutoTlist(self):
        traj_len = 0
        for i in range(len(self.points) - 1):
            vec = self.points[i + 1] - self.points[i]
            traj_len += np.linalg.norm(vec)
        t_range = traj_len / self.v
        t_list = np.array([0])
        for i in range(len(self.points) - 1):
            vec = self.points[i + 1] - self.points[i]
            vec_len = np.linalg.norm(vec)
            t_temp = vec_len / traj_len * t_range
            t_list = np.append(t_list, t_temp + t_list[-1])
        return t_list

    # 更新
    def Update(self, pos, t):
        adjust_t = self.AdjustT(t)
        pos = np.array(pos)
        if adjust_t == self.AdjustT_t and np.all(pos == self.pos_record):
            return
        else:
            self.AdjustT_t = adjust_t
            self.pos_record = pos
            self.phi = self.PhiCal(pos, self.AdjustT_t)
            self.phi_jacobian = self.PhiJacobianCal(self.AdjustT_t)
            self.e_jacobian = self.EJacobianCal(self.AdjustT_t)
            self.e_hessian = self.EHessianCal(self.AdjustT_t)

    # 当t超限时，调整t
    def AdjustT(self, t):
        if self.loop_flag:
            if t <= self.t_list[-1] and t >= self.t_list[0]:
                return t
            elif t > self.t_list[-1]:
                t = self.t_list[0] + t % (self.t_list[-1] - self.t_list[0])
                return t
            elif t < self.t_list[0]:
                cycle = (self.t_list[-1] - self.t_list[0])
                cycle_amount = (self.t_list[0] - t) // cycle + 1
                t += cycle_amount * cycle
                return t
        else:
            if t < self.t_list[0]:
                return self.t_list[0]
            elif t > self.t_list[-1]:
                return self.t_list[-1]
            else:
                return t
            
    # 寻找t所在的段
    def FindIndex(self, t):
        real_t = self.AdjustT(t)
        index = self.polys[0].FindIndex(real_t, self.t_list)
        return index

    def PhiCal(self, pos, t):
        pos = np.array(pos)
        phi = [pos[i] - self.polys[i].GetPos(t) for i in range(self.dimension)]
        return np.array(phi)
            
    def PhiJacobianCal(self, t):
        jac = np.eye(self.dimension)
        jac_add = [-self.polys[i].GetD(t) for i in range(self.dimension)]
        jac_add = np.array([jac_add]).T
        jac = np.append(jac, jac_add, axis=1)
        return jac

    def EJacobianCal(self, t):
        dp = -np.array([self.polys[i].GetD(t) for i in range(self.dimension)])
        jac_add = self.phi @ dp
        e_jac = np.append(self.phi, jac_add)
        return e_jac

    def EHessianCal(self, t):
        dp = np.array([self.polys[i].GetD(t) for i in range(self.dimension)])
        dpp = np.array([self.polys[i].GetDD(t) for i in range(self.dimension)])
        he_last = np.sum(dp**2) - self.phi @ dpp

        # hes = np.eye(self.dimension)
        # hes = np.append(hes, np.array([dp]).T, axis=1)
        # he_last = np.append(dp, [he_last])
        # hes = np.append(hes, np.array([he_last]), axis=0)

        hes = np.eye(self.dimension + 1)
        hes[-1, -1] = he_last

        return hes
    
    def TangentCal(self, t):
        # 计算切线向量
        t = self.AdjustT(t)
        tan = np.array([self.polys[i].GetD(t) for i in range(self.dimension)])
        tan = tan / np.linalg.norm(tan)
        return tan
    
    def TangentVCal(self, t):
        # 计算切线向量
        t = self.AdjustT(t)
        tan = np.array([self.polys[i].GetD(t) for i in range(self.dimension)])
        return tan

    def GetDesireP(self, t):
        pos = np.array([self.polys[k].GetPos(t)
                        for k in range(self.dimension)])
        return pos

    def GetPhi(self, pos, t):
        self.Update(pos, t)
        return self.phi

    def GetPhiJacobian(self, pos, t):
        self.Update(pos, t)
        return self.phi_jacobian
    
    def GetEJacobian(self, pos, t):
        self.Update(pos, t)
        return self.e_jacobian
    
    def GetEHessian(self, pos, t):
        self.Update(pos, t)
        return self.e_hessian
    
    def GetSampleTraj(self, point_amount=200):
        t_list = np.linspace(self.t_list[0], self.t_list[-1], point_amount)
        traj = np.zeros([0, self.dimension])
        for i in t_list:
            pos = self.GetDesireP(i)
            traj = np.append(traj, [pos], axis=0)
        traj = np.append(traj, np.array([t_list]).T, axis=1)
        return traj
    
    def CurveCal(self, t):
        # 计算kappa值和tau值
        tau = self.AdjustT(t)
        dtau = self.polys[0].dTCal(tau)
        dtau = np.reshape(dtau, [-1, 1])
        ddtau = self.polys[0].ddTCal(tau)
        ddtau = np.reshape(ddtau, [-1, 1])
        dddtau = np.array([0., 0., 0., 6])
        dddtau = np.reshape(dddtau, [-1, 1])
        M, x = self.polys[0].GetMandPara(tau)
        x = np.reshape(x, [-1, 1])
        _, y = self.polys[1].GetMandPara(tau)
        y = np.reshape(y, [-1, 1])
        _, z = self.polys[2].GetMandPara(tau)
        z = np.reshape(z, [-1, 1])

        Ai = M @ np.concatenate((x, y, z), axis=1)

        Mt_temp = M.T @ dtau @ ddtau.T @ M
        Mt = Mt_temp - Mt_temp.T

        i = y.T @ Mt @ z
        j = z.T @ Mt @ x
        k = x.T @ Mt @ y
        cross_product = np.array([np.squeeze(i), np.squeeze(j), np.squeeze(k)])
        cross_product = np.reshape(cross_product, [-1, 1])
        cross_product_len = np.linalg.norm(cross_product)
        # print("self.t_list: ", self.t_list)
        if cross_product_len <= 0.1**15:
            print("here")
            return 0
        curvey = dddtau.T @ Ai @ cross_product / cross_product_len**3
        # print(cross_product)

        kappa = cross_product_len / np.linalg.norm(dtau.T @ Ai)**3
        B = cross_product / cross_product_len
        return np.squeeze(kappa), np.squeeze(curvey), np.squeeze(B)
    
    def CurveCal2(self, t):
        # 计算kappa值和tau值
        t = self.AdjustT(t)
        dx = np.array([self.polys[i].GetD(t) for i in range(3)])
        dx = np.squeeze(dx)
        ddx = np.array([self.polys[i].GetDD(t) for i in range(3)])
        ddx = np.squeeze(ddx)
        dddx = np.array([self.polys[i].Get3D(t) for i in range(3)])
        dddx = np.reshape(dddx, [-1, 1])
        cross_product = np.reshape(np.cross(dx, ddx), [-1, 1])
        cross_len = np.linalg.norm(cross_product)
        tau = (cross_product.T / cross_len) @ (dddx / cross_len) # * np.linalg.norm(dx)
        # tau = cross_len / np.linalg.norm(dx)**2
        return tau
    
    def ThetaCal(self, theta_0, t_now, dt):
        # 积分计算theta值，注意，此处的t是指参数轨迹的参数，不是时间
        def get_d_theta(t_now):
            # kappa, d_theta, _ = self.CurveCal(t_now)
            d_theta = self.CurveCal2(t_now)
            return -d_theta
        theta_add, err = integrate.quad(get_d_theta, t_now, t_now + dt)
        # print("theta_add: ", theta_add)
        # print("err: ", err)
        theta = theta_add + theta_0
        return theta
        

if __name__ == "__main__":
    import plotly.graph_objects as go
    # points = np.array([[0., 0., 50.],
    #                    [200., 0., 200.],
    #                    [200., 500., 50.],
    #                    [150., 500., 30.],
    #                    [150., 1000., 10.],
    #                    [0., 1000., 0.],])
    # points = np.array([[0., 0., 50.],
    #                    [2000., 0., 200.],
    #                    [2000., -500., 50.],
    #                    [1500., 500., -200.],
    #                    [1500., -1000., 10.],
    #                    [0., 1000., 0.],])
    
    
    # points = np.array([[0., 0., 100.],
    #                    [0., 500., 50.],
    #                    [50., 0., 0.],])
    points = np.array([[0., 0., 0.],
                       [400., 0., 0.],
                       [400., 1000., 0.],
                       [800., 1000., 0.],
                       [800., 0., 0.],
                       [1200., 0., 0.],
                       [1200., 1200., 0.],
                       [1200., 2400., 0.],
                       [800., 2400., 0.],
                       [800., 1400., 0.],
                       [400., 1400., 0.],
                       [400., 2400., 0.],
                       [0., 2400., 0.],
                       [0., 1200., 0.]])
    
    points = np.array([[0., 0., 0.],
                       [400., 0., 0.],
                       [400., 1000., 0.],
                       [800., 1000., 0.],
                       [800., 0., 0.],
                       [1200., 0., 0.],
                       [1200., 1200., 0.],
                       [1200., 2400., 0.],
                       [800., 2400., 0.],
                       [800., 1400., 0.],
                       [400., 1400., 0.],
                       [400., 2400., 0.],
                       [0., 2400., 0.],
                       [0., 1200., 0.]])

    
    # num_points = 5
    # x_coords = np.random.randint(0, 5001, size=num_points)
    # y_coords = np.random.randint(0, 5001, size=num_points)
    # z_coords = np.zeros(num_points)
    
    # points = np.column_stack((x_coords, y_coords, z_coords))

    # num_points = 8
    # np.random.seed(2)
    # points = generate_ordered_path_points(num_points)
    # vel = 80*np.ones(num_points)
    traj = PolynomialTraj3Order(3, points, loop_flag=1)


    sample_traj = traj.GetSampleTraj()
    plt = go.Figure()
    # plt.add_trace(go.Scatter3d(x=sample_traj[:, 0],
    #                            y=sample_traj[:, 1],
    #                            z=sample_traj[:, 2],
    #                            mode='lines',
    #                            line=dict(width=3, dash='solid', color='red')))
    # plt.add_trace(go.Scatter3d(x=points[:, 0],
    #                            y=points[:, 1],
    #                            z=points[:, 2],
    #                            mode='markers',
    #                            marker=dict(size=4)))
    
    plt.add_trace(go.Scatter(x=sample_traj[:, 0],
                               y=sample_traj[:, 1],
                               mode='lines',
                               line=dict(width=2, dash='solid', color='red'),
                               name='轨迹'))
    plt.add_trace(go.Scatter(x=points[:, 0],
                               y=points[:, 1],
                               mode='markers',
                               marker=dict(size=12, symbol='x',color='blue'),
                               name='路径点'))
    
    # Adding start and end markers
    plt.add_trace(go.Scatter(x=[points[0, 0]],
                            y=[points[0, 1]],
                            mode='markers',
                            marker=dict(size=12, symbol='circle', color='green', line=dict(width=2, color='darkgreen')),
                            name='起始点'))

    plt.add_trace(go.Scatter(x=[points[-1, 0]],
                            y=[points[-1, 1]],
                            mode='markers',
                            marker=dict(size=12, symbol='triangle-up', color='orange', line=dict(width=2, color='darkorange')),
                            name='结束点'))
    
    x_min = min(sample_traj[:, 0])-200
    x_max = max(sample_traj[:, 0])+200
    y_min = min(sample_traj[:, 1])-200
    y_max = max(sample_traj[:, 1])+200
    plt.update_layout(title=dict(
                      text='多项式轨迹',
                      x=0.5,  
                      y=0.92,
                        font=dict(
                            family='宋体',  # Set font to SimSun (宋体)
                            size=20  # Adjust the size as needed
                        )
                      ),
                      xaxis_title='x/m',
                      yaxis_title='y/m',
                      xaxis=dict(range=[x_min, x_max]),
                      yaxis=dict(range=[y_min, y_max]),
                      scene=dict(aspectmode='data'),
                      showlegend=True,
                      legend=dict(
                            x=1,
                            y=1,  # Adjust y to position legend above the plot
                            xanchor='right',
                            yanchor='top',
                            orientation='h',
                            font=dict(
                                family='宋体',  # Set font to SimSun (宋体)
                                size=12  # Adjust the size as needed
                                )
                        )
                      )
    
    plt.show()
        
