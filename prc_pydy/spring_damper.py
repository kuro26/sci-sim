import sympy as sym
import sympy.physics.mechanics as me

x, v = me.dynamicsymbols('x v')
m, c, k, g, t = sym.symbols('m c k g t')

ceiling = me.ReferenceFrame('C')   # 天花板坐标系


