# 双足机器人模型计算
import sympy as sym
import sympy.physics.mechanics as me


def biped_model():
    # 定义状态变量
    alpha, beta, gamma = me.dynamicsymbols('alp beta gama')     # 与地面接触点状态
    dalpha, dbeta, dgamma = me.dynamicsymbols('alp beta gama', 1)
    lj1, lj2, lj3 = me.dynamicsymbols('lj1 lj2 lj3')            # 左腿状态变量
    dlj1, dlj2, dlj3 = me.dynamicsymbols('lj1 lj2 lj3', 1)
    rj1, rj2, rj3 = me.dynamicsymbols('rj1, rj2 rj3')           # 右腿状态变量
    drj1, drj2, drj3 = me.dynamicsymbols('rj1, rj2 rj3', 1)
    # 定义常量

    # 定义坐标系
    ground = me.ReferenceFrame('ground')                 # 左右脚区别在谁与地面绑定
    axis_1 = ground.orientnew('Axis1', 'Body', [alpha, beta, gamma], '123')
    axis_2 =

