import math

# command -> (node whose parameters it sets, keys of its status kept in the telemetry)
SWITCHES = {
    'set_inspection': '/anomaly_detector_node',
    'set_people': '/person_detector_node',
}
STATUS_KEYS = ('enabled', 'state', 'error', 'model', 'interval_sec', 'queue', 'count', 'backend')


def live_parameters(command: str, args: dict) -> list:
    """Return the (parameter, value) pairs of a set_inspection / set_people command."""
    params = []
    if 'enabled' in args:
        if not isinstance(args['enabled'], bool):
            raise ValueError('enabled must be true or false')
        params.append(('enabled', args['enabled']))
    if command == 'set_inspection' and 'model' in args:
        model = args['model']
        if not isinstance(model, str) or not model.strip() or len(model) > 200:
            raise ValueError('model must be an Ollama model name')
        params.append(('model', model.strip()))
    if command == 'set_inspection' and 'interval' in args:
        interval = args['interval']
        if (
            isinstance(interval, bool)
            or not isinstance(interval, (int, float))
            or not math.isfinite(interval)
            or interval > 3600
        ):
            raise ValueError('interval: 0 continuous, 1 to 3600 s, < 0 at stop points only')
        params.append(('interval_sec', max(float(interval), -1.0)))
    if not params:
        raise ValueError('nothing to change')
    return params


def status_summary(status) -> dict | None:
    """Return the telemetry part of a detector status JSON (on/off, state, model, ...)."""
    if not isinstance(status, dict):
        return None
    return {key: status[key] for key in STATUS_KEYS if key in status}
