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
