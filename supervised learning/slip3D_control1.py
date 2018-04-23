# 控制方法来自论文：
# 《High-Speed Humanoid Running Through Control with a 3D-SLIP Model》

from . import slip3D_ex
import pandas as pd
import numpy as np


# ------------------------------------------------
#              获取控制线性梯度表格
# ------------------------------------------------
# 获取下个周期的顶点状态 [h0, vx0, vy0]
def get_next_apex_status(pair):
    sol1, sol2, sol3, sol4, foot_point = slip3D_ex.sim_cycle(pair)
    return sol4.y[:, -1][2:5]


# 对单个pair计算雅可比矩阵
def control_jac_calculation(pair):
    m_apex = get_next_apex_status(pair)
    # 计算 jac x, u
    jac_combine = np.zeros(shape=(3, 7))
    for col in range(7):
        dm_pair = np.array(pair)
        if dm_pair[col] < 10:              # 数值计算delta的选取
            dx = 0.001
        else:
            dx = dm_pair[col] * 0.001
        dm_pair[col] = dm_pair[col] + dx
        dm_apex = get_next_apex_status(dm_pair)
        res = (dm_apex - m_apex) / dx
        jac_combine[:, col] = np.array(res).transpose()
    jac_x = jac_combine[:, 0:3]
    jac_u = jac_combine[:, 3:8]
    # 计算 dot(inv(Ju), Jx)
    jac_u_2 = jac_u[:, 0:3]
    jac_u_2[:, 2] = jac_u_2[:, 2] - jac_u[:, -1]
    j_total = np.dot(np.linalg.inv(jac_u_2), jac_x)
    return j_total


# 生成控制矩阵字典
def control_jac_dic_generate():
    m_table = pd.read_csv('./data/stable_pair.csv', header=None).values
    table_len = m_table.shape[0]
    dic = {}               # 用速度来索引控制对
    for i in range(table_len):
        pair = m_table[i]
        jac = control_jac_calculation(pair)
        dic[m_table[i, 1]] = jac
    return dic


# des_vel，期望速度,仅选取表中的速度
# x_now， 当前的状态
# 状态： [h0, vx0, vy0]
def control_para_calculation(des_vel, x_now, dic):
    # 1.检查给定的期望速度值是否在范围内
    m_table = pd.read_csv('./data/stable_pair.csv', header=None).values
    vel_list = m_table[:, 1]
    if des_vel > vel_list.max() or des_vel < vel_list.min():
        print("Error, input wrong velocity!")
        return
    # 2.规范化期望速度并找到对应的pair
    m_idx = np.fabs(vel_list - des_vel).argmin()
    m_pair = m_table[m_idx]

    # 3.计算在该pair下的delta量
    delta_x = x_now - m_pair[0: 3]         # 计算当前状态对稳态的delta x
    jac = dic[m_pair[1]]
    delta_u = np.dot(jac, delta_x.transpose())
    delta_u_out = np.array(shape=(4,))
    delta_u_out[0:3] = delta_u
    delta_u_out[-1] += -delta_u[-1]
    return m_pair, delta_u_out


