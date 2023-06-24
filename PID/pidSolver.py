class PID_Solver:
    def __init__(self, Kp=0, Ki=0, Kd=0, i_thresh=None):
        # init PID params and errors
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.cur_err = 0    # current error
        self.last_err = 0   # last error
        self.sum_err = 0    # sum error
        self.i_thresh = i_thresh  # integration thresh

    def change_param(self, kp, ki, kd):
        if ki != self.Ki:
            if self.Ki != 0 and ki != 0:
                ratio = ki / self.Ki
                self.sum_err /= ratio
            else:
                self.sum_err = 0
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def solve(self, cur_err):
        # update errors
        self.last_err = self.cur_err
        self.cur_err = cur_err
        self.sum_err += cur_err

        # integration clip
        if self.i_thresh != None and abs(self.sum_err) > self.i_thresh:
            self.sum_err = self.i_thresh / abs(self.i_thresh) * self.i_thresh
        
        # solve PID
        control_value = self.Kp * self.cur_err + self.Ki * self.sum_err + self.Kd * (self.cur_err - self.last_err)
        return control_value