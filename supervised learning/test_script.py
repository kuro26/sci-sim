import pybullet as p
import time
import pybullet_data
import numpy as np


def box_inertia(shape, m):
    x, y, z = shape
    ixx = (y*y + z*z) * m / 12
    iyy = (x*x + z*z) * m / 12
    izz = (x*x + y*y) * m / 12
    return [ixx, iyy, izz]


# 这种写法不是很有效率，暂时用一下
def rotate_x(alpha):
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([[1.0, 0.0, 0.0, 0.0],
                     [0.0, ca,  -sa, 0.0],
                     [0.0, sa,   ca, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def rotate_y(beta):
    cb = np.cos(beta)
    sb = np.sin(beta)
    return np.array([[cb,  0.0,  sb, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [-sb, 0.0,  cb, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def rotate_z(gamma):
    cg = np.cos(gamma)
    sg = np.sin(gamma)
    return np.array([[cg,  -sg, 0.0, 0.0],
                     [sg,   cg, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def trans_xyz(x, y, z):
    return np.array([[1.0, 0.0, 0.0, x],
                     [0.0, 1.0, 0.0, y],
                     [0.0, 0.0, 1.0, z],
                     [0.0, 0.0, 0.0, 1.0]])


def coord_fake_world2joint(robot_id, p_world, leg):
    assert p_world.shape == (3, 1)
    tmp = p_world.tolist()
    tmp.append([1])
    pw_ext = np.array(tmp)
    b_po = p.getBasePositionAndOrientation(robot_id)
    alpha, beta, gamma = p.getEulerFromQuaternion(b_po[1])  # 机体欧拉角表示
    if leg == 'left':
        dy = 0.12
    else:
        dy = -0.12
    t1 = rotate_x(alpha) * rotate_y(beta) * trans_xyz(0, dy, -0.2)
    p_in1 = np.linalg.inv(t1).dot(pw_ext)
    ang_a = np.arctan2(p_in1[1], abs(p_in1[2]))
    t2 = rotate_x(ang_a)
    p_in2 = np.linalg.inv(t2).dot(p_in1)
    y, z = p_in2[1], p_in2[2]
    tmp1 = y * y + z * z
    tmp2 = np.sqrt(-tmp1 * (tmp1 - 1))
    ang_b = -2 * np.arctan((y + (y * y * tmp2) / tmp1 + (z * z * tmp2) / tmp1) / (tmp1 - z))
    ang_c = 2 * np.arctan(tmp2 / tmp1)
    # ang_b = 2 * np.arctan((-y + (y*y*tmp2)/tmp1 + (z*z*tmp2)/tmp1)/(tmp1 - z))
    # ang_c = -2 * np.arctan(tmp2/tmp1)
    return [ang_a, ang_b, ang_c]


g = 9.8
sim_cycle = 0.01

# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
RobotId = p.loadURDF("bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
# mode = p.VELOCITY_CONTROL

w = 1    # 转速 rad/s
mode = p.POSITION_CONTROL
r = 0.3

for i in range(1500):
    # 控制程序
    t_now = sim_cycle * i
    x = 0.0 + r * np.cos(w * t_now)
    y = 0.12 + r * np.sin(w * t_now)
    z = -0.7
    p_w = np.array([[x], [y], [z]])
    ang_a, ang_b, ang_c = coord_fake_world2joint(RobotId, p_w, 'left')
    p.setJointMotorControl2(RobotId, 0, mode, targetPosition=ang_a)
    p.setJointMotorControl2(RobotId, 1, mode, targetPosition=ang_b)
    p.setJointMotorControl2(RobotId, 2, mode, targetPosition=ang_c)
    p.stepSimulation()
    time.sleep(sim_cycle)

p.disconnect()