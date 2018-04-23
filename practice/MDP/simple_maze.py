# 一个地图不变的迷宫


class Mdp:
    def __init__(self):
        self.state = [1, 2, 3, 4, 5, 6, 7, 8]      # 系统状态

        self.terminal_states = dict()              # 结束状态集合
        self.terminal_states[6] = 1                # 这种状态定义是有冗余的
        self.terminal_states[7] = 1
        self.terminal_states[8] = 1

        self.actions = ['n', 'e', 's', 'w']        # 动作集合

        self.rewards = dict()                      # 奖励函数
        self.rewards['1_s'] = -1.0
        self.rewards['3_s'] = 1.0
        self.rewards['5_s'] = -1.0

        self.t = dict()                            # 迁移函数
        self.t['1_s'] = 6                          # 这个写法也是很不好的
        self.t['1_e'] = 2
        self.t['2_w'] = 1
        self.t['2_e'] = 3
        self.t['3_s'] = 7
        self.t['3_w'] = 2
        self.t['3_e'] = 4
        self.t['4_w'] = 3
        self.t['4_e'] = 5
        self.t['5_s'] = 8
        self.t['5_w'] = 4

    def transform(self, state, action):
        if state in self.terminal_states:
            return True, state, 0

