
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

m_apex = get_next_apex_status(m_pair)
jac_combine = np.zeros(shape=(3, 7))
for col in range(7):
    dm_pair = np.array(m_pair)
    if dm_pair[col] < 10:  # 数值计算delta的选取
        dx = 0.001
    else:
        dx = dm_pair[col] * 0.0001
    dm_pair[col] = dm_pair[col] + dx
    dm_apex = get_next_apex_status(dm_pair)
    res = (dm_apex - m_apex) / dx
    jac_combine[:, col] = np.array(res).transpose()
jac_x = jac_combine[:, 0: 3]
jac_u = jac_combine[:, 3: 7]

jac_u_2 = jac_u[:, 0:3]
print(jac_u_2)
jac_u_2[:, 2] = jac_u_2[:, 2] - jac_u[:, -1]
print(jac_u_2)

j_total = np.dot(np.linalg.inv(jac_u_2), jac_x)
print(j_total)

