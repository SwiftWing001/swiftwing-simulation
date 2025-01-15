import numpy as np


class DesirePath:
    sample_x = np.array([20., 25., 35., 40., 45.])
    sample_y = np.array([55., 70., 120., 150., 185.])
    AdvanceLenCal = np.poly1d(np.polyfit(sample_x, sample_y, 2))
    
    def __init__(self, points, desire_v, loop_flag=True):
        # points 为n x 2 矩阵，并且相邻两点之间距离需大于advance_len
        self.desire_v = desire_v
        self.advance_len = self.AdvanceLenCal(self.desire_v) * 1.2   # 提前长度
        self.loop_flag = loop_flag
        if not loop_flag:
            self.points = np.copy(points)
        else:
            # 闭合曲线
            if np.all(np.equal(points[0, :], points[-1, :])):
                self.points = np.copy(points)
            else:
                self.points = np.append(points, [points[0, :]], axis=0)
        p_amount, _ = np.shape(self.points)
        p_dis = [np.linalg.norm(self.points[i, :] - self.points[i + 1, :]) for i in range(p_amount - 1)]
        # 判断点是否满足要求
        if np.any(np.array(p_dis) < self.advance_len):
            raise ValueError("points distance must bigger than " + str(self.advance_len))

    def NearestP(self, uav_p):
        # ===输入点，在期望轨迹上计算其投影===
        for i in range(len(self.points) - 1):
            start_p = self.points[i, :]
            end_p = self.points[i + 1, :]
            dis, near_p = self.PointLineDistance(start_p, end_p, uav_p)

            if i == 0:
                min_index = i
                min_dis = dis
                min_p = np.copy(near_p)
            elif dis <= min_dis:
                min_index = i
                min_dis = dis
                min_p = np.copy(near_p)
        # if self.loop_flag and min_index == len(self.points) - 2:
        #     min_p = 0
        # --------计算提前点---------
        dis = np.linalg.norm(self.points[min_index + 1, :] - min_p)
        if dis > self.advance_len:
            pass
        elif min_index == len(self.points) - 2 and not self.loop_flag:
            pass
        else:
            unit_0 = self.points[min_index + 1, :] - self.points[min_index, :]
            unit_0 /= np.linalg.norm(unit_0)
            if min_index == len(self.points) - 2:
                min_index = 0
            else:
                min_index += 1
            add_len = self.advance_len - dis
            unit_1 = self.points[min_index + 1] - self.points[min_index]
            unit_1 /= np.linalg.norm(unit_1)
            # min_p = self.points[min_index, :] + unit * add_len
            min_p = self.points[min_index, :] - unit_0 * self.advance_len + (unit_0 + unit_1) * add_len
            print("in  ", "dis: ", dis, "   add_len: ", add_len, "   min_index: ", min_index)

        # 计算期望速度
        vec = self.points[min_index + 1, :] - self.points[min_index, :]
        vec = vec / np.linalg.norm(vec)
        desire_v = vec * self.desire_v
        return min_p, min_dis, desire_v , 0. #, dis_to_end

    def NearestWayIndex(self, uav_p):
        # ===输入点，在期望轨迹上计算其投影===
        for i in range(len(self.points) - 1):
            start_p = self.points[i, :]
            end_p = self.points[i + 1, :]
            dis, near_p = self.PointLineDistance(start_p, end_p, uav_p)

            if i == 0:
                min_index = i
                min_dis = dis
                min_p = np.copy(near_p)
            elif dis <= min_dis:
                min_index = i
                min_dis = dis
                min_p = np.copy(near_p)
        return min_index + 1
    
    def GetDesireStatus(self, uav_p):
        # ===输入点，计算期望状态（位置和速度）===
        min_p, min_dis, desire_v, dis_to_end = self.NearestP(uav_p)
        status = np.append(min_p, desire_v)
        return status
    
    @ staticmethod
    def PointLineDistance(l_start, l_end, p):
        # 寻找点到线段的最近距离和最近点
        vec_line = np.array(l_end - l_start)
        line_len = np.linalg.norm(vec_line)
        vec_line /= line_len

        vec_p = np.array(p - l_start)
        distance_to_start = vec_p @ vec_line
        nearest_p = distance_to_start * vec_line + l_start

        # 判断是否超出线段
        vec_foot_start = np.array(nearest_p - l_start)
        if vec_foot_start @ vec_line < 0:
            nearest_p = np.copy(l_start)
        elif np.linalg.norm(vec_foot_start) > line_len:
            nearest_p = np.copy(l_end)

        nearest_dis = np.linalg.norm(p - nearest_p)
        return nearest_dis, nearest_p


def LineTest():
    # ===测试文件===
    points = np.array([[1., 2.], [10., 4.], [-3., 4.], [4., 12.]])
    desire_v = 12.
    path = DesirePath(points, desire_v)
    uav_p = np.array([12., 12.])
    min_p, min_dis, desire_v, dis_to_end = path.NearestP(uav_p)
    print("min_p: ", min_p, "    min_dis: ", min_dis, "    desire_v: ", desire_v)
    for i in range(len(points) - 1):
        start_p = path.points[i, :]
        test_p = start_p + np.array([0.5, 0.5])
        min_p, min_dis, desire_v, dis_to_end = path.NearestP(test_p)
        print("dis_to_end: ", dis_to_end)

    import plotly.graph_objects as go
    plt = go.Figure()
    plt.add_trace(go.Scatter(x=path.points[:, 0],
                             y=path.points[:, 1],
                             mode='lines',
                             line=dict(width=3, dash='solid', color='blue')))
    plt.add_trace(go.Scatter(x=np.array([uav_p[0], min_p[0]]),
                             y=np.array([uav_p[1], min_p[1]]),
                             mode='lines',
                             line=dict(width=3, dash='solid', color='red')))
    plt.show()


if __name__ == "__main__":
    LineTest()
