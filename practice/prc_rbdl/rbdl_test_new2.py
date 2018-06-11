# --------------------------------------
# 本例模型： 三连杆模型
#               j2 ⊙-------- j3
#      | G        /     b3
#      ↓         / b2
#    j0   b1    /
# Base⊙-------⊙ j1
# 新增：2018年6月11日
#  1. 添加对约束力的显示
# ---------------------------------------
import numpy as np
import rbdl
import matplotlib.pyplot as plt
import matplotlib.animation as animation


l1, l2, l3 = [0.5, 0.4, 0.6]
m1, m2, m3 = [1.0, 2.0, 3.0]


model = rbdl.Model()
model.gravity = np.array([0.0, 0.0, -9.81])
# 1. 创建三个body
ixx = m1 * l1 * l1 / 12
izz = ixx
b1 = rbdl.Body.fromMassComInertia(
    m1,
    np.array([0., l1/2, 0.]),
    np.diag([ixx, 0.0, izz]))
ixx = m2 * l2 * l2 / 12
izz = ixx
b2 = rbdl.Body.fromMassComInertia(
    m2, np.array([0., 0., 0.]),
    np.diag([ixx, 0.0, izz])
)
ixx = m3 * l3 * l3 / 12
izz = ixx
b3 = rbdl.Body.fromMassComInertia(
    m3, np.array([0., l3/2, 0.]),
    np.diag([ixx, 0.0, izz])
)
# 2. 创建关节，平面浮动平台约束(q1, q2, q3)
planar_float_type = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])
joint_planar_x = rbdl.Joint.fromJointAxes(planar_float_type)
# 创建关节，x轴转动约束(j1, j2)
joint_rot_x = rbdl.Joint.fromJointType("JointTypeRevoluteX")
# 3. 向模型中添加body
trans = rbdl.SpatialTransform()
trans.r = np.array([0.0, 1.0, 1.0])        # floating base位置
b_base = model.AppendBody(trans, joint_planar_x, b1)
trans.r = np.array([0.0, -l2/2, 0.0])      # 关节1相对fb坐标系位置
b_link1 = model.AddBody(b_base, trans, joint_rot_x, b1)
trans.r = np.array([0.0, l2/2, 0.0])       # 关节2相对fb坐标系位置
b_link3 = model.AddBody(b_base, trans, joint_rot_x, b3)

# 添加约束
constrain_set_l1 = rbdl.ConstraintSet()
c_point = np.array([0.0, l1, 0.0])
# 话说这个名字参数居然不是可以不要的，虽然没什么用
constrain_set_l1.AddConstraint(b_link1, c_point, np.array([0., 1., 0.]), 'ground_y'.encode('utf-8'))
constrain_set_l1.AddConstraint(b_link1, c_point, np.array([0., 0., 1.]), 'ground_z'.encode('utf-8'))
constrain_set_l1.Bind(model)

# 基于该模型仿真测试一下
q0 = np.zeros(model.q_size)
qd0 = np.zeros(model.qdot_size)
qdd0 = np.zeros(model.qdot_size)
tau = np.zeros(model.qdot_size)
# 思考：基于约束的仿真出来约束点是否会变化，最佳的仿真效果还是从当前出发
# 不过：我们不用这个来仿真，用来只做当前的控制，是没有问题的
t_cycle = 0.002


# 产生绘图数据
def data_gen(q=q0[:], qd=qd0[:], qdd=qdd0[:]):
    t_max = 2
    for i in range(int(t_max/t_cycle)):
        # 从源文件知道，这个动力学计算是不包括约束的，所以用这个计算约束没用
        # 改了一些封装之后可用了
        rbdl.CalcContactSystemVariables(model, q, qd, tau, constrain_set_l1)
        H = constrain_set_l1.get_H()
        G = constrain_set_l1.get_G()
        C = constrain_set_l1.get_C()
        gamma = constrain_set_l1.get_gamma()
        A = np.zeros((7, 7))
        A[0:5, 0:5] = H
        A[5:7, 0:5] = G
        A[0:5, 5:7] = G.transpose()
        b = np.zeros((7, 1))
        b[0:5, 0] = -C.transpose()
        b[5:7, 0] = gamma.transpose()
        res = np.dot(np.linalg.inv(A), b)
        qdd = res[0:5, 0].transpose()
        lamb = res[5:7, 0]
        # 这种简单的积分是一定会产生漂移的，约束点的漂移简直可怕
        qd = qd + qdd * t_cycle
        q = q + qd * t_cycle
        yield q, lamb


def run(data):
    q, lamb = data
    # print(q)
    j0_p = rbdl.CalcBodyToBaseCoordinates(model, q, b_link1, np.array([0., l1, 0.]))
    j1_p = rbdl.CalcBodyToBaseCoordinates(model, q, b_link1, np.array([0., 0., 0.]))
    j2_p = rbdl.CalcBodyToBaseCoordinates(model, q, b_link3, np.array([0., 0., 0.]))
    j3_p = rbdl.CalcBodyToBaseCoordinates(model, q, b_link3, np.array([0., l3, 0.]))

    link1.set_data([j0_p[1], j1_p[1]], [j0_p[2], j1_p[2]])
    link2.set_data([j1_p[1], j2_p[1]], [j1_p[2], j2_p[2]])
    link3.set_data([j2_p[1], j3_p[1]], [j2_p[2], j3_p[2]])
    link4.set_data([j0_p[1], j0_p[1]-lamb[0]/60], [j0_p[2], j0_p[2]-lamb[1]/60])
    # gif_writer.grab_frame()



def init():
    ax.set_ylim(-2, 2)
    ax.set_xlim(-0, 4)


fig, ax = plt.subplots()
link1, = ax.plot([], [], 'b', lw=2)
link2, = ax.plot([], [], 'r', lw=2)
link3, = ax.plot([], [], 'g', lw=2)
link4, = ax.plot([], [], 'r', lw=2)
ax.grid()

# 保存为gif文件
gif_writer=animation.ImageMagickFileWriter(fps=2)
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10, repeat=False, init_func=init)
ani.save('res.gif',writer=gif_writer)
# gif_writer.finish()
plt.show()

# 测试完毕
# 1. 注意定义body时设定的基坐标，在rbdl_test中是没改过的，可以试试
# 2. 在动力学文件中的inversedynamic是不包含约束的
# 3. 简单的积分漂移真的大
