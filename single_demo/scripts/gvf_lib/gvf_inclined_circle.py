import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Cylinder():
    def __init__(self, center=[0, 0, 0], r=50., ori=2, k=[0.1,0.1,0.1]):     # ori为圆柱的朝向，0、1、2分别为x、y、z
        self.dimension = 3
        self.center = np.array(center)
        self.r = float(r)
        self.ori = ori
        self.k = k      # 增益

        if self.ori == 0:
            self.circle_index = [1, 2]
        elif self.ori == 1:
            self.circle_index = [0, 2]
        elif self.ori == 2:
            self.circle_index = [0, 1]

    # 输入位置，输出phi值
    def GetPhi(self, pos):
        pos = np.array(pos)
        center = self.center[self.circle_index]
        pos = pos[self.circle_index]
        phi = np.linalg.norm(pos - center) - self.r
        return phi
    
    # 输入位置，输出雅可比矩阵(一维)
    def GetJacobin(self, pos):
        pos = np.array(pos)
        center = self.center[self.circle_index]
        pos = pos[self.circle_index]
        vec = pos - center
        jac_temp = vec / np.linalg.norm(vec)
        jac = np.zeros([3])
        jac[self.circle_index] = jac_temp
        return jac
    

class InclinePlane():
    def __init__(self, normal_vector=[1.,1.,5.], point=[0.,0.,100.],k=[0.1,0.1,0.1]):
        # normal vector and cross point of a plane
        self.n = np.array(normal_vector)
        self.point = np.array(point)
        self.k = k
        self.CalFunction()

    def CalFunction(self):
        # function of plane: Ax + By + Cz + D =0. Normal_vector = [A, B, C]
        # output coefficient = [A, B, C, D]
        self.coefficient = np.copy(self.n)
        d = -self.n@self.point
        self.coefficient = np.append(self.coefficient, d)

    def GetPhi(self, pos):
        pos = np.array(pos)
        self.phi = self.coefficient[0:3]@pos + self.coefficient[-1]
        return self.phi

    def GetJacobin(self, pos):
        pos = np.array(pos)
        jac = self.coefficient[0:3]
        return jac


class GVF():
    def __init__(self, surfaces):
        self.surfaces = surfaces
        self.k = np.vstack([surfaces[0].k,surfaces[1].k])

    def GetPhi(self, pos):
        self.phi_1 = self.surfaces[0].GetPhi(pos)
        self.phi_2 = self.surfaces[1].GetPhi(pos)
        return self.phi_1, self.phi_2
        
    def GetJacobin(self, pos):
        self.jac_1 = self.surfaces[0].GetJacobin(pos)
        self.jac_2 = self.surfaces[1].GetJacobin(pos)
        return self.jac_1, self.jac_2
        
    def GetNormalV(self, pos):
        phi_1, phi_2 = self.GetPhi(pos)
        jac_1, jac_2 = self.GetJacobin(pos)
        print('phi_1,phi_2:',phi_1,phi_2)
        print('jac_1,jac_2:',jac_1,jac_2)
        self.vn = - self.k[0]*phi_1 * jac_1 \
                  - self.k[1]*phi_2 * jac_2
        return self.vn

    def GetTangentV(self, pos):
        jac_1, jac_2 = self.GetJacobin(pos)
        self.vt = np.cross(jac_1, jac_2)
        return self.vt
    
    def GetGuideV(self, pos):
        vg = self.GetTangentV(pos) + self.GetNormalV(pos)
        vg /= np.linalg.norm(vg)
        self.vg = vg
        return self.vg

if __name__ == "__main__":
    plane = InclinePlane(normal_vector=[1.,1.,5.], point=[0.,0.,100.],k=[0.1,0.1,0.1])
    cylinder = Cylinder(center=[0, 0, 0], r=50., ori=2, k=[0.1,0.1,0.1])
    gvf = GVF(surfaces=[plane,cylinder])
    
    gvf.GetGuideV([0.,1.,1.])
    print(gvf.vg)
