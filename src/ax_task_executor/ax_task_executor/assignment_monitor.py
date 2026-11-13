"""Renew D4 assignment checks only while a trusted A3 task is active."""
from copy import deepcopy
import threading


class AssignmentMonitor:
    """Bind periodic checks and permission invalidation to one recorded task."""

    def __init__(self, authority, send_check, period_s=0.5):
        self.authority = authority
        self.send_check = send_check
        self.period_s = period_s
        self.lock = threading.RLock()
        self.execution = None
        self.next_check = 0.0

    def start(self, execution):
        """Start after the real dispatcher records acceptance, never for a mock."""
        if (not execution.execution_id or not execution.assignment_id
                or execution.assignment_generation <= 0):
            raise ValueError('recorded execution and assignment required')
        with self.lock:
            if self.execution is not None:
                raise ValueError('stop the current task before tracking another')
            self.execution = deepcopy(execution)
            self.next_check = 0.0
            self.tick()

    def tick(self):
        """Request checks on BOOTTIME and latch expiry before any recovery."""
        with self.lock:
            if self.execution is None:
                return
            self.authority.expire_assignment()
            now = self.authority.clock()
            if now >= self.next_check:
                self.send_check(self.execution)
                self.next_check = now + self.period_s

    def stop(self, execution_id):
        """Invalidate before ending tracking; ignore a late prior task completion."""
        with self.lock:
            if self.execution is None or self.execution.execution_id != execution_id:
                return False
            self.authority.invalidate_assignment()
            self.execution = None
            return True
