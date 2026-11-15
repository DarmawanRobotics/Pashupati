from __future__ import annotations

from pashupati_inspection.console import nodes

from dharma_utils.config import build


def test_console_lists_every_node_with_valid_defaults():
    data = nodes()
    assert {n['node'] for n in data['nodes']} == {'anomaly_detector_node', 'person_detector_node'}
    for node in data['nodes']:
        for cls, prefix in node['configs']:
            build(cls, {}, prefix)
