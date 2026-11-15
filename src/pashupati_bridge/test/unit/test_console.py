from __future__ import annotations

from pashupati_bridge.console import nodes

from dharma_utils.config import build


def test_console_lists_every_node_with_valid_defaults():
    data = nodes()
    assert {n['node'] for n in data['nodes']} == {
        'teleop_udp_node',
        'rtsp_camera',
        'video_stream_node',
    }
    for node in data['nodes']:
        for cls, prefix in node['configs']:
            build(cls, {}, prefix)
