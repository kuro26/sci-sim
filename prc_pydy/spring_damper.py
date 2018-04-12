import sympy as sym
import sympy.physics.mechanics as me

# 定义变量
x, v = me.dynamicsymbols('x v')
m, c, k, g, t = sym.symbols('m c k g t')

# 天花板参考坐标系
ceiling = me.ReferenceFrame('C')
# 创建两个点
O = me.Point('O')
P = me.Point('P')
# 将O固定在天花板上
O.set_vel(ceiling, 0)
# 设置P点位置，以及速度表达
P.set_pos(O, x * ceiling.x)
P.set_vel(ceiling, v * ceiling.x)
P.vel(ceiling)
# 给出力的表达式
damping = -c * P.vel(ceiling)
stiffness = -k * P.pos_from(O)
gravity = m * g * ceiling.x
forces = damping + stiffness + gravity
# 直接牛顿第二定律
# acc可以根据前面的表述给出加速度
zero = me.dot(forces - m * P.acc(ceiling), ceiling.x)
# 求解加速度和速度
dv_by_dt = sym.solve(zero, v.diff(t))[0]
dx_by_dt = v

# 2. 或者说直接用kane法进行求解
mass = me.Particle('mass', P, m)   # 添加质量点
kane = me.KanesMethod(ceiling, q_ind=[x], u_ind=[v], kd_eqs=[v - x.diff(t)])
# 添加质点上的力和质量，注意外力是tuple类型的数组
fr, frstar = kane.kanes_equations([mass], [(P, forces)])
M = kane.mass_matrix_full
f = kane.forcing_full



