"""The NETRA uplink for the Dharma console (menuconfig of dharma_console)."""

from __future__ import annotations

from pashupati_fleet.core.config import FleetConfig


def nodes() -> dict:
    """Return the uplink node with the configuration it declares."""
    node = {'node': 'fleet_uplink_node', 'section': 'NETRA uplink', 'configs': [(FleetConfig, '')]}
    return {'package': 'pashupati_fleet', 'defaults': ['config/fleet.yaml'], 'nodes': [node]}
