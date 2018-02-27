"""一个简单的离散PID实现"""


# 仿真时间是每一个周期update的，最好只用于仿真中确定时间间隔的
# 非实时应用
class DisPid:
    def __init__(self, p=0.2, i=0.0, d=0.0, gap=0.005):
        self.Kp = p
        self.Ki = i
        self.Kd = d

        self.t_cycle = gap        # 采样周期
        self.out = 0.0            # 控制输出
        self.errInt = 0.0         # 误差积分
        self.err_1 = 0.0

        self.integrator_windup = 20.0  # 积分饱和上限
        self.int_err_max = 10           # 积分分离上限
        self.max_u = 10                # 输出上限

    # PID 控制器处理的输入应该为error
    # 离散PID 没有连续时间的概念，只有时间间隔
    def update(self, err):
        # 比例项
        p_item = self.Kp * err
        # 积分项
        self.errInt += err
        if self.errInt > self.integrator_windup:   # 积分饱和处理
            self.errInt = self.integrator_windup
        coefficient_i = 1                          # 积分分离处理
        if err > self.int_err_max:
            coefficient_i = 0                      # 针对具体系统可以设计渐变系数
        i_item = self.Ki * self.errInt * self.t_cycle * coefficient_i
        # 微分项
        d_item = self.Kd * (err - self.err_1)/self.t_cycle

        u = p_item + i_item + d_item
        if u > self.max_u:
            self.out = self.max_u
        else:
            self.out = u

        # 善后
        self.err_1 = err

    def set_p(self, p=0.2):
        self.Kp = p

    def set_i(self, i=0.0):
        self.Ki = i

    def set_d(self, d=0.0):
        self.Kd = d

    def controller_reset(self):
        self.out = 0
        self.err_1 = 0.0
        self.errInt = 0.0
