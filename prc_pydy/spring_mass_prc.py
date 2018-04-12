# 自己独立练习的
import sympy as sym
import sympy.physics.mechanics as me

# 定义变量
x, v = me.dynamicsymbols('x v')
m, c, k, g, t = sym.symbols('m c k g t')

# 定义坐标系
ceiling = me.ReferenceFrame('Ceil')
#  定义关键点
O = me.Point('O')
P = me.Point('P')
# 定义点与坐标系的关系
O.set_vel(ceiling, 0)
P.set_pos(O, x*ceiling.x)
P.set_vel(ceiling, v*ceiling.x)
# 外力的表达式，kane方程中将所有的重力视为外力
damping = -c * P.vel(ceiling)     # 与天花板的速度
stiffness = -k * P.pos_from(O)    # 与O点的距离
gravity = m * g * ceiling.x       # 重力方向为x
forces = damping + stiffness + gravity
# 添加质量点
mass = me.particle('mass', P, m)
# Kane方法求解
kane = me.KanesMethod(ceiling, q_ind=[x],u_ind=[v], kd_eqs=[v-x.diff(t)])
fr, frstar = kane.kanes_equations([mass], [(P, forces)])
M = kane.mass_matrix_full
f = kane.forcing_full



