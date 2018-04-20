import numpy as np
from .spatialTransform import Transform


class Body:
    def __init__(self, mass=0.,
                 com=np.array([0., 0., 0.]),
                 inertia=np.ones((3, 3))):
        self.mass = mass
        self.com = com.reshape((3, 1))
        self.inertia = inertia

    # 将新的body固连到这个body，主要用于修改本body质量参数
    # transform, 转移矩阵表达
    # n_body, 要连接的另一个body
    def join(self, trans, n_body):
        new_mass = self.mass + n_body.mass
        c2_in1 = np.dot(trans.E, n_body.com) + trans.r
        new_com = (self.mass * self.com + n_body.mass * n_body.com)/(self.mass + n_body.mass)

