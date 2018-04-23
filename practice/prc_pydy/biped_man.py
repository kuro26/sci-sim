# 双足机器人模型计算
import sympy as sym
import sympy.physics.mechanics as me


# 左脚着地时系统模型
def biped_model_left():
    # 定义状态变量
    alpha, beta, gamma = me.dynamicsymbols('alp beta gama')     # 与地面接触点状态
    dalpha, dbeta, dgamma = me.dynamicsymbols('alp beta gama', 1)
    lj1, lj2, lj3 = me.dynamicsymbols('lj1 lj2 lj3')            # 左腿状态变量
    dlj1, dlj2, dlj3 = me.dynamicsymbols('lj1 lj2 lj3', 1)
    rj1, rj2, rj3 = me.dynamicsymbols('rj1, rj2 rj3')           # 右腿状态变量
    drj1, drj2, drj3 = me.dynamicsymbols('rj1, rj2 rj3', 1)
    # 定义常量

    # 定义坐标系
    ground = me.ReferenceFrame('ground')
    axis_1 = ground.orientnew('Axis1', 'Body', [alpha, beta, gamma], '123')
    axis_2 = axis_1.orientnew('Axis2', 'Axis', [lj2, axis_1.y])

    # 定义点
    ori = me.Point('ori')



