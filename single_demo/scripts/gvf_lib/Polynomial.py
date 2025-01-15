import numpy as np


# 多项式模板类，实现轨迹、导数、二阶导的计算
# 子类需要实现矩阵M的计算
class PolynomialTemplate:
    def __init__(self, order, key_points, start_t, t_list) -> None:
        self.order = order                      # 阶数
        self.key_points = key_points            # 关键点    2维，self.seg_amount+1 x 2*self.order-1
        self.seg_amount = len(self.key_points) - 1  # 段数      
        self.t_start = start_t                  # t的起始点 
        # self.t_step = t_step                    # t的步进值
        # self.t_list = np.linspace(0, self.seg_amount, self.seg_amount + 1) *\
        #     self.t_step + self.t_start          # 关键点对应的时间 
        self.t_list = t_list

        self.M = self.MCal()                    # 系数的解矩阵，需由子类实现，2维，2*(self.order-1) x (self.order+1)
        self.A_list = self.ACal()               # 系数矩阵列表  2维，(self.order+1) x self.seg_amount

        # ---FindIndex()函数暂存的值---
        self.FindIndexLast_t = self.t_list[-1]
        self.FindIndexLast_index = -1
        self.FindIndexLast_index = self.FindIndex(self.t_start, self.t_list)

    def MCal(self):
        raise AttributeError("Class must realize function 'self.MCal'!")
    
    def GetPos(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        seg_index = self.FindIndex(t, self.t_list)
        x = np.dot(self.A_list[[seg_index]], self.TCal(t))
        return np.squeeze(x)
    
    def GetD(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        seg_index = self.FindIndex(t, self.t_list)
        dx = np.dot(self.A_list[[seg_index]], self.dTCal(t))
        return np.squeeze(dx)
        
    def GetDD(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        seg_index = self.FindIndex(t, self.t_list)
        ddx = np.dot(self.A_list[[seg_index]], self.ddTCal(t))
        return np.squeeze(ddx)
    
    def Get3D(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        seg_index = self.FindIndex(t, self.t_list)
        dddT = np.array([0, 0, 0, 6])
        dddx = np.dot(self.A_list[[seg_index]], dddT)
        return np.squeeze(dddx)
        
    def ACal(self):
        # raise AttributeError("Class must realize function 'self.ACal'!")
        A = np.zeros([0, self.order + 1])
        for i in range(self.seg_amount):
            p_list = np.append(self.key_points[i], self.key_points[i + 1])
            p_list = np.array([p_list]).T

            A_i = self.M[i] @ p_list
            A = np.append(A, A_i.T, axis=0)
        return A

    def TCal(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        tau = t - self.t_list[self.FindIndex(t, self.t_list)]
        T = np.array([[tau ** (i) for i in range(self.order + 1)]])
        return T.T
    
    def dTCal(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        tau = t - self.t_list[self.FindIndex(t, self.t_list)]
        temp_dT = np.array([(i + 1) * tau ** (i) for i in range(self.order)])
        dT = np.array([np.append(0, temp_dT)])
        return dT.T
    
    def ddTCal(self, t):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        tau = t - self.t_list[self.FindIndex(t, self.t_list)]
        temp_ddT = np.array([(i + 1) * (i + 2) * tau**i for i in range(self.order - 1)])
        ddT = np.array([np.append([0, 0], temp_ddT)])
        return ddT.T
    
    def FindIndex(self, t, t_list):
        if t < self.t_list[0] or t > self.t_list[-1]:
            raise ValueError("The t is out of range!")
        # 寻找t在t_list中属于第几段
        if t == self.FindIndexLast_t:
            return self.FindIndexLast_index
        equal = np.where(t_list == t)
        if len(equal[0]) != 0:
            if equal[0][0] == len(t_list) - 1:
                self.FindIndexLast_t = t
                self.FindIndexLast_index = equal[0][0] - 1
                return self.FindIndexLast_index
            else:
                self.FindIndexLast_t = t
                self.FindIndexLast_index = equal[0][0]
                return self.FindIndexLast_index
        low_index = np.where(t_list < t)
        high_index = np.where(t_list > t)
        if high_index[0][0] - low_index[0][-1] == 1:
            self.FindIndexLast_t = t
            self.FindIndexLast_index = low_index[0][-1]
            return self.FindIndexLast_index
        else:
            raise ValueError("Some thing wrong in t_list" + str(t_list))


class Polynomial3Order(PolynomialTemplate):
    def __init__(self, key_points, t_list) -> None:
        super().__init__(order=3, key_points=key_points, start_t=0, t_list=t_list)

    def MCal(self):
        M = np.zeros([0, self.order + 1, self.order * 2 - 2])
        for i in range(len(self.t_list) - 1):
            tau = self.t_list[i + 1] - self.t_list[i]
            M_temp = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [-3 / tau**2, -2 / tau, 3 / tau**2, -1 / tau],
                               [2 / tau**3, 1 / tau**2, -2 / tau**3, 1 / tau**2]])
            M = np.append(M, [M_temp], axis=0)
        return M

    def GetMandPara(self, t):
        # 输入参数返回对应的矩阵M和[x(0),v(0),x(1),v(1)]
        index = self.FindIndex(t, self.t_list)
        M = self.M[index]
        para = np.append(self.key_points[index], self.key_points[index + 1])
        return M, para

    
