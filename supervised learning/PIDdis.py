
"""一个简单的离散PID实现"""
import time


# 仿真时间是每一个周期update的，最好只用于仿真中确定时间间隔的
class DisSimPid:
    def __init__(self, p=0.2, i=0.0, d=0.0, gap=0.005):
        self.Kp = p
        self.Ki = i
        self.Kd = d

        self.sample_time = gap   # 采样周期
        self.cur_time = 0.0      # 当前时间
        self.last_time = 0.0     # 上一时刻时间
        self.ref = 0.0           # 参考输入
        self.out = 0.0           # 控制输出

        # 积分饱和参数
        self.windup_guard = 20.0    # 积分饱和上限

    def controller_init(self):
        self.ref = 0
        self.out = 0

    def update(self, err):

