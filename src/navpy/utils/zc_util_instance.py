import threading


class ZcUtilInstance:
    """
    Thread-safe singleton wrapper for ZcUtil.
    Usage:
        zc = ZcUtilInstance.get()                 # default args
        zc = ZcUtilInstance.get(max_distance=500) # first call sets args
    """
    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get(cls, *args, **kwargs):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:  # double-checked locking
                    from navpy.modules.navigation.geo.zc_util import ZcUtil
                    cls._instance = ZcUtil(*args, **kwargs)
        return cls._instance
