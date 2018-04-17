# --------------------------------------
# 本例模型： 三连杆模型
#               j2 ⊙-------- j3
#      | G        /     b3
#      ↓         / b2
#    j0   b1    /
# Base⊙-------⊙ j1
# ---------------------------------------
import numpy as np
import rbdl


l1, l2, l3 = [0.5, 0.5, 0.6]
m1, m2, m3 = [1.0, 2.0, 3.0]


model = rbdl.Model()
model.gravity = np.array([0.0, 0.0, -9.81])
# 1. 创建三个body
ixx = m1 * l1 * l1 / 12
izz = ixx
b1 = rbdl.Body.fromMassComInertia(
    double_mass=m1,      # 这里不太能理解，pyx文件中是没有cls_type参数的
    ndarray_com=np.array([0., 0., 0.]),
    ndarray_inertia=np.diag([ixx, 0.0, izz]))
ixx = m2 * l2 * l2 / 12
izz = ixx
b2 = rbdl.Body.fromMassComInertia(
    m2, np.array([0., 0., 0.]),
    np.diag([ixx, 0.0, izz])
)
ixx = m3 * l3 * l3 / 13
izz = ixx
b3 = rbdl.Body.fromMassComInertia(
    m3, np.array([0., 0., 0.]),
    np.diag([ixx, 0.0, izz])
)
# 2. 创建关节，平面浮动平台约束(q1, q2, q3)
planar_float_type = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0]])
joint_planar_x = rbdl.Joint.fromJointAxes(planar_float_type)
# 创建关节，x轴转动约束(j1, j2)
joint_rot_x = rbdl.Joint.fromJointType("JointTypeRevoluteX")
# 3. 向模型中添加body
# 机体初始位置
trans = rbdl.SpatialTransform()
trans.r = np.array([0.0, 1.0, 1.0])
base = model.AppendBody(trans, joint_planar_x, b1)
trans.r = np.array([])

















