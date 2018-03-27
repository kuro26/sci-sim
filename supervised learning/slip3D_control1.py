# 控制方法来自论文：
# 《High-Speed Humanoid Running Through Control with a 3D-SLIP Model》

import slip3D_ex
import pandas as pd
import numpy as np


# ------------------------------------------------
#              获取控制线性梯度表格
# des_vel，期望速度,仅选取表中的速度
# x_now， 当前的状态
# 状态： [h0, vx0, vy0]
# ------------------------------------------------
def get_next_apex_status(pair):
    sol1, sol2, sol3, sol4, foot_point = slip3D_ex.sim_cycle(pair)
    return sol4.y[:, -1][2:5]


def conrol_jac_calculation(pair):
    m_apex = get_next_apex_status(pair)
    # jac x的计算
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


def control_table_establish():
    m_table = pd.read_csv('./data/stable_pair.csv', header=None).values
    vel_list = m_table[:, 1]

def control_para_calculation(des_vel, x_now):
    # 1.检查给定的期望速度值是否在范围内
    m_table = pd.read_csv('./data/stable_pair.csv', header=None).values
    vel_list = m_table[:, 1]
    if des_vel > vel_list.max() or des_vel < vel_list.min():
        print("Error, input wrong velocity!")
        return
    # 2.规范化期望速度并找到对应的pair
    m_idx = np.fabs(vel_list - des_vel).argmin()
    m_pair = m_table[m_idx]

    # 3.计算梯度


