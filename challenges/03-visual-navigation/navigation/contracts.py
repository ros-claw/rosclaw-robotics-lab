"""Capture identity and reset boundaries for observations."""

from dataclasses import dataclass


class ObservationRejected(ValueError):
    pass


@dataclass(frozen=True)
class CaptureIdentity:
    epoch: int
    sequence: int
    stamp_ns: int


class CaptureEpoch:
    def __init__(self):
        self.epoch = 0
        self.clock_ns = None
        self.last_capture_ns = None
        self.sequence = 0

    def observe_clock(self, stamp_ns):
        rewound = self.clock_ns is not None and stamp_ns < self.clock_ns
        if rewound:
            self.epoch += 1
            self.last_capture_ns = None
            self.sequence = 0
        self.clock_ns = stamp_ns
        return rewound

    def accept_capture(self, stamp_ns, epoch):
        if epoch != self.epoch:
            raise ObservationRejected("observation belongs to a previous epoch")
        if not self.clock_ns or not 0 < stamp_ns <= self.clock_ns:
            raise ObservationRejected("capture stamp is outside the observed clock")
        if self.last_capture_ns is not None and stamp_ns <= self.last_capture_ns:
            raise ObservationRejected("repeated or out-of-order capture stamp")
        self.last_capture_ns = stamp_ns
        self.sequence += 1
        return CaptureIdentity(self.epoch, self.sequence, stamp_ns)


class StopDwell:
    """Accumulate low-speed evidence over distinct, fresh odometry samples."""

    max_gap_ns = 300_000_000
    required_ns = 750_000_000

    def __init__(self):
        self.start_ns = None
        self.last_ns = None

    def observe(self, clock_ns, stamp_ns, linear_mps, angular_rps):
        if stamp_ns > clock_ns:
            # Clock and odometry are different DDS topics.
            return False
        if clock_ns - stamp_ns >= self.max_gap_ns:
            self.start_ns = None
            return False
        if stamp_ns == self.last_ns:
            return False
        if self.last_ns is not None and not 0 < stamp_ns - self.last_ns < self.max_gap_ns:
            self.start_ns = None
        self.last_ns = stamp_ns
        if not (0 <= linear_mps <= 0.03 and abs(angular_rps) <= 0.08):
            self.start_ns = None
            return False
        if self.start_ns is None:
            self.start_ns = stamp_ns
        return stamp_ns - self.start_ns >= self.required_ns
