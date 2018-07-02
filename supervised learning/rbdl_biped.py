# ----------------------------------------
# 文件描述：基于BipedRobotOne.urdf的机器人模型
# 需求说明：修改过的rbdl-python
# 测试参考：操作空间复现文档.docx
# 结果：在
# ----------------------------------------
import numpy as np
import rbdl
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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
    lid = [id_torso,
           id_left_link1,id_left_thigh, id_left_shank,
           id_right_link1, id_right_thigh, id_right_shank]
    # 添加部件的id_list

    return model, cs_left, cs_right, lid


def modelTest():
    t_cycle = 0.001
    model, cs_left, cs_right, lid = biped_model_create()
    cs_left.Bind(model)
    id_torso, idl_link, idl_thigh, idl_shank, idr_link, idr_thigh, idr_shank = lid
    # 初始化数据
    fig, ax = plt.subplots()
    AB, = ax.plot([], [], 'b', lw=2)
    FC, = ax.plot([], [], 'r', lw=2)
    CD, = ax.plot([], [], 'g', lw=2)
    DE, = ax.plot([], [], 'b', lw=2)
    FG, = ax.plot([], [], 'g', lw=2)
    GH, = ax.plot([], [], 'b', lw=2)
    ax.grid()
    # 动画绘图初始化函数
    def init():
        ax.set_ylim(-5,1)
        ax.set_xlim(-3,3)

    def data_gen():
        t_max = 2
        # 设计关节力为0
        q0 = np.zeros(model.q_size)
        print(model.q_size)
        dq0 = np.zeros(model.qdot_size)
        tau = np.zeros(model.qdot_size)

        # 循环初始化
        q, dq = q0, dq0
        for i in range(int(t_max / t_cycle)):
            rbdl.CalcContactSystemVariables(model, q, dq, tau, cs_left)
            H = cs_left.get_H()
            G = cs_left.get_G()
            C = cs_left.get_C()
            gamma = cs_left.get_gamma()
            sz_d1 = H.shape[0] + G.shape[0]
            A = np.zeros((sz_d1, sz_d1))
            # 构造混合的A矩阵
            A[0:H.shape[0], 0:H.shape[0]] = H
            A[H.shape[0]:sz_d1, 0:H.shape[0]] = G
            A[0:H.shape[0], H.shape[0]:sz_d1] = G.transpose()
            b = np.zeros((sz_d1, 1))
            b[0:H.shape[0], 0] = -C.transpose()
            b[H.shape[0]:sz_d1, 0] = gamma.transpose()
            # 计算动力学方程
            res = np.dot(np.linalg.inv(A), b)
            ddq = res[0:H.shape[0], 0].transpose()
            fc = res[H.shape[0]:sz_d1, 0]
            # 还是这个简单的积分吧
            dq = dq + ddq * t_cycle
            q = q + dq * t_cycle
            yield q, fc

    def run(data):
        show_axis = 'yz'
        q, fc = data
        pA = rbdl.CalcBodyToBaseCoordinates(model, q, id_torso, np.array([0., 0., 0.2]))
        pB = rbdl.CalcBodyToBaseCoordinates(model, q, id_torso, np.array([0., 0., -0.2]))
        pC = rbdl.CalcBodyToBaseCoordinates(model, q, idl_link, np.array([0., 0., 0.]))
        pD = rbdl.CalcBodyToBaseCoordinates(model, q, idl_thigh, np.array([0., 0., -0.5]))
        pE = rbdl.CalcBodyToBaseCoordinates(model, q, idl_shank, np.array([0., 0., -0.5]))
        pF = rbdl.CalcBodyToBaseCoordinates(model, q, idr_link, np.array([0., 0., 0.]))
        pG = rbdl.CalcBodyToBaseCoordinates(model, q, idr_thigh, np.array([0., 0., -0.5]))
        pH = rbdl.CalcBodyToBaseCoordinates(model, q, idr_shank, np.array([0., 0., -0.5]))
        if show_axis=='yz':
            AB.set_data([pA[1], pB[1]], [pA[2], pB[2]])
            FC.set_data([pF[1], pC[1]], [pF[2], pC[2]])
            CD.set_data([pC[1], pD[1]], [pC[2], pD[2]])
            DE.set_data([pD[1], pE[1]], [pD[2], pE[2]])
            FG.set_data([pF[1], pG[1]], [pF[2], pG[2]])
            GH.set_data([pG[1], pH[1]], [pG[2], pH[2]])
        else:
            AB.set_data([pA[0], pB[0]], [pA[2], pB[2]])
            FC.set_data([pF[0], pC[0]], [pF[2], pC[2]])
            CD.set_data([pC[0], pD[0]], [pC[2], pD[2]])
            DE.set_data([pD[0], pE[0]], [pD[2], pE[2]])
            FG.set_data([pF[0], pG[0]], [pF[2], pG[2]])
            GH.set_data([pG[0], pH[0]], [pG[2], pH[2]])
    ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10, repeat=False, init_func=init)
    plt.show()


# modelTest()







