#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

class circle():
    def __init__(self, center=np.array([0., 0.]), r=10, k=1):
        self.center = center
        self.r = float(r)
        self.k_init = float(k)
        self.k = self.k_init

    def update_phi(self, pos):
        pos = np.array(pos)
        self.phi = np.sum((pos - self.center)**2)**0.5 - self.r

    def update_error(self,pos):
        pos = np.array(pos)
        self.update_phi(pos)
        # define the error function here
        self.error = self.phi

    def get_grad(self, pos):
        pos = np.array(pos)
        diff = pos - self.center
        norm = np.linalg.norm(diff)
        if norm == 0:  
            return np.array([0., 0.])
        grad = diff / norm
        self.grad = grad
        return grad

    def get_tau(self, pos, rotate="cw"):
        if rotate == 'cw':
            R = np.array([[0, 1], [-1, 0]])
        if rotate == 'ccw':
            R = np.array([[0, -1], [1, 0]])
        grad = self.get_grad(pos)
        tau = np.dot(R, grad)
        self.tau = tau
        return tau

    def get_omega(self, pos):
        self.update_error(pos)
        # if self.error >=10:
        #     self.k = 3.
        # else:
        #     self.k = self.k_init
        # self.k = 0.1 * self.error
        # if self.error <= 10:
        #     self.k = 1.
        # print('增益k=',self.k,'误差e=',self.error)
        omega = self.get_tau(pos)-self.k * self.error * self.get_grad(pos)
        if np.linalg.norm(omega) == 0:
            return np.array([0., 0.])
        else:
            normalized_omega = omega / np.linalg.norm(omega)
        return np.array(normalized_omega)
    

if __name__ == "__main__":
    k_values = [0.2, 1.0, 2.0, 4.0]
    x, y = np.meshgrid(np.arange(-20, 21, 1.), np.arange(-20, 21, 1.))
    fig, axes = plt.subplots(2, 2, figsize=(10,10))

    for idx, k_value in enumerate(k_values):
        c = circle(k=k_value)
        U = np.zeros_like(x)
        V = np.zeros_like(y)

        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                pos = np.array([x[i, j], y[i, j]]).astype(float)
                U[i, j], V[i, j] = c.get_omega(pos)

        ax = axes[idx // 2, idx % 2]
        ax.quiver(x, y, U, V, color='black', scale=50, width=0.002)
        path = plt.Circle((0, 0), 10, color='black', fill=False, alpha=0.8)
        ax.add_artist(path)
        ax.set_title(f'k = {k_value}')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.set_aspect('equal')

    plt.tight_layout()
    plt.show()