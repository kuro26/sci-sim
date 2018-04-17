import numpy as np
import rbdl

model = rbdl.Model()
joint_rot_y = rbdl.Joint.fromJointType("JointTypeRevoluteY")
# 创建机体1
body = rbdl.Body.fromMassComInertia(
    1.0,
    np.array([0., 0.5, 0.]),
    np.eye(3) * 0.05)

# 创建空间转移矩阵，并设平移为[0, 1, 0]
xtrans = rbdl.SpatialTransform()
xtrans.r = np.array([0., 1., 0.])
print(joint_rot_y)
print(model)
print(body)
print(body.mInertia)
print(xtrans)
# 三杆模型，第一个body，绕ry转动
body_1 = model.AppendBody(rbdl.SpatialTransform(), joint_rot_y, body)
body_2 = model.AppendBody(xtrans, joint_rot_y, body)
body_3 = model.AppendBody(xtrans, joint_rot_y, body)
# 获取模型中变量
q = np.zeros(model.q_size)
qdot = np.zeros(model.qdot_size)
qddot = np.zeros(model.qdot_size)
tau = np.zeros(model.qdot_size)
# 设置当前角度
q[0] = 1.3
q[1] = -0.5
q[2] = 3.2
# 计算运动学关系
point_local = np.array([1., 2., 3.])     # 一个局部坐标值
point_base = rbdl.CalcBodyToBaseCoordinates(model, q, body_3, point_local)    # 在body3的局部坐标系下的的点在全局中的坐标
point_local_2 = rbdl.CalcBaseToBodyCoordinates(model, q, body_3, point_base)  # 反过来
# 前向动力学
rbdl.ForwardDynamics(model, q, qdot, tau, qddot)
print("qddot = " + str(qddot.transpose()))

G = np.zeros([3, model.qdot_size])

rbdl.CalcPointJacobian(model, q, body_3, point_local, G)

print("G = \n" + str(G))
/