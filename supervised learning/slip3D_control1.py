# 控制方法来自论文：
# 《High-Speed Humanoid Running Through Control with a 3D-SLIP Model》

import slip3D_ex
import pandas as pd
import math


# ------------------------------------------------
#              在简化模型层给出控制参数
# des_vel，期望速度,仅选取表中的速度
# x_now， 当前的状态
# 状态： [h0, vx0, vy0]
# ------------------------------------------------
def control_para_calculation(des_vel, x_now):
    m_table = pd.read_csv('./data/stable_pair.csv', header=None).values
    vel_list = m_table[:, 1]
    if des_vel > vel_list.max() or des_vel < vel_list.min():
        print("Error, input wrong velocity!")
        return
    else:
        des_vel1 = math.ceil(des_vel)



