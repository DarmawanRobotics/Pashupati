"""The inspection nodes for the Dharma console (menuconfig of dharma_console)."""

from __future__ import annotations

from pashupati_inspection.core.config import AnomalyConfig, PeopleConfig


def nodes() -> dict:
    """Return every node with the configuration it declares."""
    configs = {
        'anomaly_detector_node': [(AnomalyConfig, '')],
        'person_detector_node': [(PeopleConfig, '')],
    }
    return {
        'package': 'pashupati_inspection',
        'defaults': ['config/inspection.yaml'],
        'nodes': [{'node': n, 'section': 'Inspection', 'configs': c} for n, c in configs.items()],
    }
