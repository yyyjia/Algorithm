'''
    PID控制算法基本理论：
        在线性控制系统中，才可利用PID控制算法加以控制，其中线性指系统具有“齐次性”和“叠加性”
        整个控制系统主要有以下几大部分：
            目标输出target、控制器、执行器、对象、实际输出cur_feedback
        常用系统为：开环系统、单闭环控制系统、双闭环系统
        
        常用基本公式：
            pid_out = Kp * Error + Ki * sum(Error) + Kd * (Error_cur - Error_last)
        其中，Kp为比例控制系数，Ki为积分控制系数，Kd为微分控制系数；在整个控制系统中，为了避免系统大震动的出现，可
        进行积分限幅、积分分离、微分先行等操作............ 
'''

import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline


class PID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

def test_pid(P, I , D, L):

    pid = PID(P, I, D)

    pid.SetPoint=1.1
    pid.setSampleTime(0.01)

    END = L
    feedback = 0
    feedback_list = []
    time_list = []
    setpoint_list = []

    for i in range(1, END):
        pid.update(feedback)
        output = pid.output
        feedback += output
        time.sleep(0.01)
        feedback_list.append(feedback)
        setpoint_list.append(pid.SetPoint)
        time_list.append(i)

    time_sm = np.array(time_list)
    time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
    # feedback_smooth = spline(time_list, feedback_list, time_smooth)
    feedback_smooth = make_interp_spline(time_list, feedback_list)(time_smooth)
    plt.figure(0)
    plt.grid(True)
    plt.plot(time_smooth, feedback_smooth,'b-')
    plt.plot(time_list, setpoint_list,'r')
    plt.xlim((0, L))
    plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('PythonTEST PID',fontsize=15)

    plt.ylim((1-0.5, 1+0.5))

    plt.grid(True)
    plt.show()

# if __name__ == "__main__":
#     test_pid(0.9, 0.8, 0.0001, L=100)
    '''
                PID调试的一般原则：
            
            在输出不震荡时，增大比例增益；
            
            在输出不震荡时，减少积分时间常数；
            
            在输出不震荡时，增大微分时间常数；
            
            PID调节口诀：
            
            参数整定找最佳，从小到大顺序查
            
            先是比例后积分，最后再把微分加
            
            曲线振荡很频繁，比例度盘要放大
            
            曲线漂浮绕大湾，比例度盘往小扳
            
            曲线偏离回复慢，积分时间往下降
            
            曲线波动周期长，积分时间再加长
            
            曲线振荡频率快，先把微分降下来
            
            动差大来波动慢，微分时间应加长
            
            理想曲线两个波，前高后低四比一
            
            一看二调多分析，调节质量不会低
    '''