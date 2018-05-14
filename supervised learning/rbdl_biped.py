import numpy as np
import rbdl


# 创建模型
def biped_model_create():
    model = rbdl.Model()
    model.gravity = np.array([0.0, 0.0, -9.81])
    thigh_len = 0.5
    shank_len = 0.5
    # 1. 创建body,全部写下来是为了方便调试
    # 左右腿一般是对称的
    # 非精准的建模也是允许的
    mass, com = 20.0, np.array([0., 0., 0.])  # 躯干
    inertia = np.diag([0.37, 0.29, 0.128])
    b_torso = rbdl.Body.fromMassComInertia(mass, com, inertia)
    mass, com = 0.5, np.array([0., 0., 0.])  # 连接件1
    inertia = np.diag([0.0003, 0.0003, 0.0003])
    b_link1 = rbdl.Body.fromMassComInertia(mass, com, inertia)
    mass, com = 2.0, np.array([0., 0., -thigh_len / 2])  # 大腿
    inertia = np.diag([0.04208, 0.04208, 0.00083])
    b_thigh = rbdl.Body.fromMassComInertia(mass, com, inertia)
    mass, com = 2.0, np.array([0., 0., -shank_len / 2])  # 小腿
    inertia = np.diag([0.04208, 0.04208, 0.00083])
    b_shank = rbdl.Body.fromMassComInertia(mass, com, inertia)
    # 允许存在误差，不对足端进行建模
    # mass, com = 0.5, np.array([0., 0., 0.])                 # 足--看能不能固定关节
    # inertia = np.diag([0.0001, 0.0001, 0.0001])
    # b_foot = rbdl.Body.fromMassComInertia(mass, com, inertia)

    # 2. 创建运动副
    space_free_type = np.diag([1, 1, 1, 1, 1, 1])
    joint_float_space = rbdl.Joint.fromJointAxes(space_free_type)
    joint_rot_x = rbdl.Joint.fromJointType("JointTypeRevoluteX")
    joint_rot_y = rbdl.Joint.fromJointType("JointTypeRevoluteY")

    # 3. 向模型中添加body
    trans = rbdl.SpatialTransform()  # 躯干
    trans.r = np.array([0.0, 0.0, 0.0])
    id_torso = model.AppendBody(trans, joint_float_space, b_torso)
    trans.r = np.array([0.0, 0.12, -0.2])  # 左连接件
    id_left_link1 = model.AddBody(id_torso, trans, joint_rot_x, b_link1)
    trans.r = np.array([0.0, 0.0, 0.0])  # 左大腿
    id_left_thigh = model.AddBody(id_left_link1, trans, joint_rot_y, b_thigh)
    trans.r = np.array([0.0, 0.0, -thigh_len])  # 左小腿
    id_left_shank = model.AddBody(id_left_thigh, trans, joint_rot_y, b_shank)
    trans.r = np.array([0.0, -0.12, -0.2])  # 右连接件
    id_right_link1 = model.AddBody(id_torso, trans, joint_rot_x, b_link1)
    trans.r = np.array([0.0, 0.0, 0.0])  # 右大腿
    id_right_thigh = model.AddBody(id_right_link1, trans, joint_rot_y, b_thigh)
    trans.r = np.array([0.0, 0.0, -thigh_len])  # 右小腿
    id_right_shank = model.AddBody(id_right_thigh, trans, joint_rot_y, b_shank)

    # 4. 设计约束
    cs_left = rbdl.ConstraintSet()  # 左腿约束
    c_point = np.array([0.0, 0.0, -0.55])
    cs_left.AddConstraint(id_left_shank, c_point, np.array([1., 0., 0.]), 'ground_x'.encode('utf-8'))
    cs_left.AddConstraint(id_left_shank, c_point, np.array([0., 1., 0.]), 'ground_y'.encode('utf-8'))
    cs_left.AddConstraint(id_left_shank, c_point, np.array([0., 0., 1.]), 'ground_z'.encode('utf-8'))
    cs_right = rbdl.ConstraintSet()  # 右腿约束
    c_point = np.array([0.0, 0.0, -0.55])
    cs_right.AddConstraint(id_right_shank, c_point, np.array([1., 0., 0.]), 'ground_x'.encode('utf-8'))
    cs_right.AddConstraint(id_right_shank, c_point, np.array([0., 1., 0.]), 'ground_y'.encode('utf-8'))
    cs_right.AddConstraint(id_right_shank, c_point, np.array([0., 0., 1.]), 'ground_z'.encode('utf-8'))
    return model, cs_left, cs_right

