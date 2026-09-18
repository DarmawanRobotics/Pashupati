from robot_navigation.utils.avoidance.braitenberg_avoidance import BraitenbergAvoidance
from robot_navigation.utils.avoidance.vfh_avoidance import VfhAvoidance

AVOIDANCE_ALGORITHMS = {
    'braitenberg': BraitenbergAvoidance,
    'vfh': VfhAvoidance,
}


def create_avoidance(name: str, params: dict):
    """Build an avoidance algorithm instance by name using the given constructor kwargs."""
    if name not in AVOIDANCE_ALGORITHMS:
        raise ValueError(f'unknown avoidance algorithm: {name!r}, options are {list(AVOIDANCE_ALGORITHMS)}')
    return AVOIDANCE_ALGORITHMS[name](**params)
