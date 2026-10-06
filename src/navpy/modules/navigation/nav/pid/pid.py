import logging
from collections import namedtuple
from time import time

from navpy.args.pid_args import PIDArgs


class Pid(object):
    PIDParams = namedtuple('PIDParams', ['Kp', 'Ki', 'Kd'])

    def __init__(self, args: PIDArgs, out_min, out_max, set_point=0):
        self.args = args

        self._logger = logging.getLogger(type(self).__name__)
        self._Kp = args.kp
        self._Ki = args.ki
        self._Kd = args.kd
        self._set_point = set_point
        self._output_limits = [out_min, out_max]

        self._integral = 0.0
        self._last_error = 0.0
        self._last_measurement = 0.0
        self.last_time = time()

        self.reset(out_min, out_max)

    def calc(self, measurement):
        current_time = time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 1.0  # Prevent division by zero or negative time intervals

        # Compute all the working error variables
        error = self._set_point - measurement
        d_input = measurement - self._last_measurement

        # Proportional term
        p = self._Kp * error

        # Integral term with anti-windup: Only integrate if the process is not saturated
        self._integral = self._bound(self._integral + self._Ki * error * dt)
        i = self._integral

        # Derivative term (on measurement, to avoid derivative kick)
        d = -self._Kd * d_input / dt

        # Compute PID Output
        output = self._bound(p + i + d)

        # Remember some variables for next time
        self._last_error = error
        self._last_measurement = measurement
        self.last_time = current_time

        return output

    def reset(self, out_min, out_max):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_measurement = 0.0
        self.last_time = time()

        self._Kp = self.args.kp
        self._Ki = self.args.ki
        self._Kd = self.args.kd
        self._output_limits = [out_min, out_max]

    def set_tunings(self, kp, ki, kd):
        self._Kp = kp
        self._Ki = ki
        self._Kd = kd

    def get_params(self):
        return Pid.PIDParams(self._Kp, self._Ki, self._Kd)

    def set_kp(self, kp):
        self._Kp = kp

    def set_ki(self, ki):
        self._Ki = ki

    def set_kd(self, kd):
        self._Kd = kd

    def _bound(self, value):
        if self._output_limits[0] is not None and self._output_limits[1] is not None:
            return min(max(value, self._output_limits[0]), self._output_limits[1])
        return value
