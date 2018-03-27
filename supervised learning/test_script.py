
import slip3D_ex
import pandas as pd
import numpy as np


def get_next_apex_status(pair):
    sol1, sol2, sol3, sol4, foot_point = slip3D_ex.sim_cycle(pair)
    return sol4.y[:, -1][2:5]


m_table = pd.read_csv('./data/stable_pair.csv', header=None).values
vel_list = m_table[:, 1]
m_idx = np.fabs(vel_list - 4.25).argmin()
m_pair = m_table[m_idx]
# Jacobian x的计算
jac_x = np.zeros(shape=(3, 3))
for col in range(3):
    dm_pair = np.array(m_pair)
    if dm_pair[col] < 10:
        dx = 0.001
    else:
        dx = dm_pair[col] * 0.001
    dm_pair[col] = dm_pair[col] + dx
    m_apex = get_next_apex_status(m_pair)
    dm_apex = get_next_apex_status(dm_pair)
    res = (dm_apex - m_apex) / dx
    print('col:', col)
    print(res)
    jac_x[:, col] = np.array(res).transpose()
    print(jac_x)

# Jacobian u的计算
jac_u = np.zeros(shape=(3, 4))
col_list = [3, 4, 5, 6]
for col in col_list:
    dm_pair = np.array(m_pair)
    if dm_pair[col] < 10:
        dx = 0.001
    else:
        dx = dm_pair[col] * 0.001
    dm_pair[col] = dm_pair[col] + dx
    m_apex = get_next_apex_status(m_pair)
    dm_apex = get_next_apex_status(dm_pair)
    res = (dm_apex - m_apex) / dx
    print('col:', col)
    print(res)
    jac_u[:, col-3] = np.array(res).transpose()
    print(jac_u)
