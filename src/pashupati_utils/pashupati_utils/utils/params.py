"""Setting another node's parameters and reading service answers."""

from __future__ import annotations

from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


def typed_parameter(name: str, value) -> Parameter:
    """Return a Parameter; a list is a string array, never empty (rclpy: byte array)."""
    if isinstance(value, (list, tuple)):
        return Parameter(name, Parameter.Type.STRING_ARRAY, [str(v) for v in value] or [''])
    return Parameter(name, value=value)


def set_parameters_request(pairs: list) -> SetParameters.Request:
    """Return a SetParameters request for (name, value) pairs (types from the values)."""
    return SetParameters.Request(
        parameters=[typed_parameter(name, value).to_parameter_msg() for name, value in pairs]
    )


def outcome(result) -> tuple[bool, str]:
    """Return (ok, message) of a service response: success / message or SetParameters results."""
    if result is None:
        return False, 'no response'
    results = getattr(result, 'results', None)
    if results is not None:
        refused = [r.reason or 'refused' for r in results if not r.successful]
        return not refused, '; '.join(refused)
    return bool(getattr(result, 'success', True)), str(getattr(result, 'message', ''))
