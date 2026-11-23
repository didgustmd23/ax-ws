"""D1 input freshness only; no localization decisions or actuation permissions."""
import math


class InputWatchdog:
    # D-05 health lifetime; scan lifetime matches existing D1 costmap setting.
    LIMITS = {'localization': 1.0, 'front': .5, 'rear': .5}

    def __init__(self):
        self.samples = {}
        self.last_ros = None

    def observe(self, source, stamp, now_boot, valid=True):
        self.samples[source] = (stamp, now_boot, valid)

    def reason(self, now_ros, now_boot):
        if not math.isfinite(now_ros) or now_ros <= 0:
            return 'ROS_TIME_INVALID'
        if self.last_ros is not None and now_ros < self.last_ros:
            self.samples.clear()
            self.last_ros = now_ros
            return 'ROS_TIME_REWIND'
        self.last_ros = now_ros
        for source, lifetime in self.LIMITS.items():
            sample = self.samples.get(source)
            if sample is None:
                return source.upper() + '_MISSING'
            stamp, received, valid = sample
            if not valid:
                return source.upper() + '_INVALID'
            if not all(math.isfinite(v) for v in (stamp, received, now_boot)):
                return source.upper() + '_TIME_INVALID'
            if stamp <= 0 or stamp > now_ros + .05 or received > now_boot:
                return source.upper() + '_TIME_INVALID'
            if now_ros - stamp > lifetime or now_boot - received > lifetime:
                return source.upper() + '_STALE'
        return ''
