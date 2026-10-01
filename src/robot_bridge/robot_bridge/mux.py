MODES = ('auto', 'remote')


class ControlMux:
    """Chooses which velocity command reaches the robot: navigation (auto) or teleop (remote)."""

    def __init__(self, mode: str = 'auto', remote_timeout: float = 0.3):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.mode = mode
        self._remote_timeout = remote_timeout
        self._remote_stamp = None
        self._stale_sent = True

    def set_mode(self, mode: str) -> bool:
        """Switch mode; returns True when it changed."""
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        changed = mode != self.mode
        self.mode = mode
        self._remote_stamp = None
        self._stale_sent = False
        return changed

    def on_nav(self, cmd):
        """Return the command to forward for a navigation command, or None to drop it."""
        return cmd if self.mode == 'auto' else None

    def on_remote(self, cmd, now: float):
        """Return the command to forward for a teleop command, or None to drop it."""
        if self.mode != 'remote':
            return None
        self._remote_stamp = now
        self._stale_sent = False
        return cmd

    def watchdog(self, now: float) -> bool:
        """Return True once when teleop went silent in remote mode, or right after a switch."""
        if self._stale_sent:
            return False
        if self.mode == 'remote' and self._remote_stamp is not None:
            if now - self._remote_stamp < self._remote_timeout:
                return False
        self._stale_sent = True
        return True
