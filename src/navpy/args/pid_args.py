from navpy.utils.arg_helper import StoreWithFlag


class PIDArgs(object):
    def __init__(self, args, prefix, param_getter=None, param_kp_name=None):
        self.kp = None
        self.ki = None
        self.kd = None

        self.args = args
        self.prefix = prefix
        self._param_getter = param_getter
        self._param_kp_name = param_kp_name

        self.refresh()

    def refresh(self):
        self.kp = getattr(self.args, f'{self.prefix}_kp')
        self.ki = getattr(self.args, f'{self.prefix}_ki')
        self.kd = getattr(self.args, f'{self.prefix}_kd')

        overrides = getattr(self.args, "_cli_overrides", set())
        if self._param_getter and self._param_kp_name and f'{self.prefix}_kp' not in overrides:
            kp_override = self._param_getter(self._param_kp_name)
            if kp_override is not None:
                kp_override = float(kp_override)
                self.kp = kp_override
                setattr(self.args, f'{self.prefix}_kp', kp_override)

    @staticmethod
    def add_args(parser, prefix):
        args = parser.add_argument_group(f"{prefix} Controller arguments")

        if prefix == 'pitch':
            kp = 1.2
            ki = 0.0
            kd = 0.00
        else:
            kp = 0.0
            ki = 0.0
            kd = 0.0

        args.add_argument(f"-{prefix[0]}kp", f"--{prefix}-kp", type=float, default=kp, action=StoreWithFlag,
                          help=f"Proportional gain for {prefix} controller (default: {kp})")
        args.add_argument(f"-{prefix[0]}ki", f"--{prefix}-ki", type=float, default=ki,
                          help=f"Integral gain for {prefix} controller (default: 0.0)")
        args.add_argument(f"-{prefix[0]}kd", f"--{prefix}-kd", type=float, default=kd,
                          help=f"Differential gain for {prefix} controller (default: 0.0)")
