import numpy as np


class RunLogger:
    def __init__(self):
        self.data = {
            "t": [],
            "h": [],
            "v": [],
            "a": [],
            "u_lag": [],
            "u_cmd": [],
            "pred_no": [],
            "pred_full": [],
            "err": [],
            "enabled": [],
            "u_ff": [],
            "u_smc": [],
            "u_trim": [],
            "s": [],
            "mach": [],
            "F_drag": [],

            # --- timing ---
            "iter_time_ms": [],   # time taken for one loop iteration (ms)
            "elapsed_s": [],      # cumulative elapsed wall time since start (s)
            "rt_margin_ms": [],   # (dt - iter_time) in ms, if dt known
        }

    def push(self, **kwargs):
        for k, v in kwargs.items():
            if k in self.data:
                self.data[k].append(v)

    def to_numpy(self):
        return {k: np.array(v, dtype=float) for k, v in self.data.items()}
