class ControllerBusyError(RuntimeError):
    """Raised when a controller is already owned by another caller."""


class ExclusiveController:
    """Mixin that allows only one logical caller to command a controller."""

    def __init__(self, name):
        self._controller_name = name
        self._owner = None

    @property
    def owner(self):
        return self._owner

    def acquire(self, owner):
        owner = _normalize_owner(owner)
        if self._owner is None:
            self._owner = owner
            return True
        return self._owner == owner

    def release(self, owner):
        owner = _normalize_owner(owner)
        if self._owner == owner:
            self._owner = None
            return True
        return False

    def require_owner(self, owner):
        owner = _normalize_owner(owner)
        if self._owner is None:
            self._owner = owner
        if self._owner != owner:
            raise ControllerBusyError(
                '%s is owned by %s, not %s' %
                (self._controller_name, self._owner, owner)
            )
        return owner


def _normalize_owner(owner):
    if owner is None:
        raise ValueError('controller owner is required')
    owner = str(owner).strip()
    if not owner:
        raise ValueError('controller owner cannot be empty')
    return owner

