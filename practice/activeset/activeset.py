# -----------------------------------------
# 文件说明：使用active-set方法进行QP问题求解
# QP问题基本格式：
#      min 0.5*x'*H*x + cx
#      s.t. Ax >=b
# 但是我要处理的不是这种很通用的问题，是特殊问题；
# 所以，我应该首先明确我们的问题是什么
# -----------------------------------------
import numpy as np


def active_set_1(mH, vc, mA, vb):
    len_x = mH.size[0]                  # 变量数
    x0 = np.zeros((len_x,))
    act_c = [0]
    step_count = 0
    while(1):
        step_count = step_count + 1
        # 求解在当前约束集下
        if(len(act_c) == 0):
            lamb = 0



