# 贝塞尔曲线的实现，将作为本次实验的轨迹表达方式
# 当然，当你发现原来就有轮子的时候，你的心情是不那么好的，NURBS-python
# pip install NURBS-Python 即可安装对应的库
import numpy as np
import scipy.special as sp


# m---贝塞尔曲线的阶数，控制点系数数量为阶数+1
# n---曲线的数量
# 一维贝塞尔曲线
class BezierD1:
    def __init__(self, m, n):
        self.m = m
        self.n = n
        self.coeff = np.ones((n, m+1))
        self.begin = 0                 # 默认曲线起点
        self.end = 1                   # 默认曲线终点

    def setBezierBeginEnd(self, begin, end):
        self.begin = begin
        self.end = end

    def setControlPoint(self, coefficient):
        if coefficient.shape == (self.n, self.m+1):
            self.coeff = coefficient
            return True
        else:
            # 在小程序中没有必要使用异常处理
            print("Error: wrong size of coefficient")
            return False

    # 输入s为[0~1]范围中的
    def value(self, s, diff=0):
        # check input
        if s.max()> 1 or s.min()<0 or len(s.shape)>1:
            print("Error: input index is out of range")
            return False, np.array([])
        else:
            val = np.zeros((self.n,len(s)))
            for i in range(self.n):
                if diff == 0:                            # 原函数值
                    for k in range(self.m + 1):
                        thisCoef = self.coeff[i, k] * sp.comb(self.m, k)
                        val[i, :] += thisCoef * np.power(s, k) * np.power(1 - s, self.m - k)
                if diff == 1:                            # 一阶导数
                    for k in range(self.m):
                        thisCoef = (self.coeff[i, k+1] - self.coeff[i, k]) * sp.comb(self.m-1, k) * self.m
                        val[i, :] += thisCoef * np.power(s, k) * np.power(1 - s, self.m - k-1)
                if diff == 2:                            # 二阶导数（这个计算方法不好）
                    for k in range(self.m+1):
                        thisCoef = self.coeff[i, k]*sp.comb(self.m, k)
                        if k == 0:
                            val[i, :] += thisCoef * self.m*(self.m-1)*np.power((1-s), (self.m-2))
                        elif k == 1:
                            val[i, :] += thisCoef * (-2*(self.m-1)*np.power((1-s), (self.m-2))
                                                     + (self.m-1)*(self.m-2)*s*np.power((1-s), (self.m-3)))
                        elif k == self.m-1:
                            val[i, :] += thisCoef * ((self.m-1)*(self.m-2)*np.power(s, (self.m-3))
                                                     - self.m*(self.m-1) * np.power(s, (self.m-2)))
                        elif k == self.m:
                            val[i, :] += thisCoef * k*(k-1)*np.power(s,(k-2))
                        else:
                            val[i, :] += thisCoef * (k*(k-1)*np.power(s, (k-2))*np.power((1-s),(self.m-k))
                                                     - 2*(self.m-k)*k*np.power(s,(k-1))*np.power((1-s),(self.m-k-1))
                                                     + (self.m-k)*(self.m-k-1)*np.power(s,k)*np.power((1-s), (self.m-k-2)))
            return True, val










