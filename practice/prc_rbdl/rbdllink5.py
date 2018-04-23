# --------------------------------------
# 本例模型： RBDL描述的link5模型
#                     /
#                    / l1
#                   /
#                  ⊙
#      | G        / \
#      ↓         /   \ l2
#               /     \
#       -------⊙      ⊙
#                     /
#                    / l3
#                   /
# ---------------------------------------
import numpy as np
import rbdl

l1, l2, l3 = [0.5, 0.5, 0.5]
model = rbdl.Model()
model.gravity = np.array([0.0, 0.0, -9.81])
# 平面浮动基座关节类型
planar_float_type = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0]])
# X平面约束关节类型
joint_planar_x = rbdl.Joint.fromJointAxes(planar_float_type)





















